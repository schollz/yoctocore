#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/flash.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/pll.h"
#include "hardware/rtc.h"
#include "hardware/structs/clocks.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "pico/binary_info.h"
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "pico/types.h"

bool usb_midi_present = false;
#ifdef INCLUDE_MIDI
#include "bsp/board.h"
#include "tusb.h"
//
#include "lib/midi_comm.h"
#endif

// utility functions
#define util_clamp(x, a, b) ((x) > (b) ? (b) : ((x) < (a) ? (a) : (x)))

#define linlin(x, xmin, xmax, ymin, ymax)                                 \
  util_clamp((ymin + (x - xmin) * (ymax - ymin) / (xmax - xmin)), (ymin), \
             (ymax))

#define linexp(x, xmin, xmax, ymin, ymax)                                     \
  util_clamp(((ymin) *                                                        \
              powf(((ymax) / (ymin)), (((x) - (xmin)) / ((xmax) - (xmin))))), \
             (ymin), (ymax))

#define BLOCKS_PER_SECOND SAMPLE_RATE / SAMPLES_PER_BUFFER
static const uint32_t PIN_DCDC_PSM_CTRL = 23;
#define DURATION_HOLD 500
#define DURATION_HOLD_LONG 1250
#define FLASH_TARGET_OFFSET (5 * 256 * 1024)

//
#include "ff.h" /* Obtains integer types */
//
#include "diskio.h" /* Declarations of disk functions */
//
#include "f_util.h"
#include "hw_config.h"
#include "my_debug.h"
#include "sd_card.h"
//
#include "lib/WS2812.h"
#include "lib/adsr.h"
#include "lib/dac.h"
#include "lib/filterexp.h"
#include "lib/knob_change.h"
#include "lib/libmidi.h"
#include "lib/luavm.h"
#include "lib/mcp3208.h"
#include "lib/memusage.h"
#include "lib/pcg_basic.h"
#include "lib/random.h"
#include "lib/scales.h"
#include "lib/sdcard.h"
#include "lib/simpletimer.h"
#include "lib/spectra.h"
#include "lib/spiral.h"
#include "lib/yoctocore.h"
#include "uart_rx.pio.h"
//

Yoctocore yocto;
DAC dac;
WS2812 ws2812;
SimpleTimer pool_timer[16];
KnobChange pool_knobs[8];
MCP3208 mcp3208;
bool blink_on = false;
const uint8_t button_num = 9;
const uint8_t button_pins[9] = {1, 8, 20, 21, 22, 26, 27, 28, 29};
uint8_t button_values[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
uint32_t time_per_iteration = 0;
uint32_t timer_per[32];
uint32_t lfo_ct_last[8] = {0, 0, 0, 0, 0, 0, 0, 0};
float lfo_index_acc[8] = {0, 0, 0, 0, 0, 0, 0, 0};

#if CFG_TUD_CDC
char cdc_rx_buffer[CFG_TUD_CDC_RX_BUFSIZE];
#endif

// const clockDivisions = [
//   "/512", "/256", "/128", "/64", "/32", "/16", "/8", "/4", "/2", "x1", "x2",
//   "x3", "x4", "x6", "x8", "x12", "x16", "x24", "x48"
// ];

const float division_values[19] = {
    1.0f / 512.0f, 1.0f / 256.0f, 1.0f / 128.0f, 1.0f / 64.0f, 1.0f / 32.0f,
    1.0f / 16.0f,  1.0f / 8.0f,   1.0f / 4.0f,   1.0f / 2.0f,  1.0f,
    2.0f,          3.0f,          4.0f,          6.0f,         8.0f,
    12.0f,         16.0f,         24.0f,         48.0f};

#ifdef INCLUDE_MIDI
#include "lib/midi_comm.h"
#include "lib/midicallback.h"
#endif

void update_linked_outs(bool triggering_outs[], bool trigger, uint32_t ct) {
  for (uint8_t i2 = 0; i2 < 8; i2++) {
    Config *config = &yocto.config[yocto.i][i2];
    Out *out = &yocto.out[i2];

    if (config->linked_to < 1)                     continue;
    if (! triggering_outs[config->linked_to - 1])  continue;

    if (config->mode == MODE_ENVELOPE) {
      // trigger the envelope
      printf("[out%d] env_off linked to out%d\n", i2 + 1,
             config->linked_to);
      ADSR_gate(&out->adsr, trigger, ct);
    } else if (config->mode == MODE_GATE) {
      // trigger the gate
      printf("[out%d] gate_off linked to out%d\n", i2 + 1,
             config->linked_to);
      if (trigger) {
        out->voltage_set = config->max_voltage;
      } else {
        out->voltage_set = config->min_voltage;
      }
    }
  }
}

void on_successful_lua_callback(int i, float volts, bool trigger) {
  uint32_t ct = to_ms_since_boot(get_absolute_time());
  bool shift = button_values[8];
  Out *out = &yocto.out[i];
  bool triggering_outs[8] = { false };

  out->voltage_set = volts;

  // find any linked outputs and activate the envelope
  triggering_outs[i] = true;
  update_linked_outs(triggering_outs, trigger, ct);
}

void timer_callback_sample_knob(bool on, int user_data) {
  for (uint8_t i = 0; i < 8; i++) {
    int16_t val_changed =
        KnobChange_update(&pool_knobs[i], MCP3208_read(&mcp3208, i, false));
    if (val_changed != -1) {
      printf("Knob %d: %d\n", i, val_changed);
      Config *config = &yocto.config[yocto.i][i];
      if (config->mode == MODE_CODE) {
        float volts;
        bool trigger;
        bool shift = button_values[8];
        float val = val_changed/1023.0f;
        printf("Lua on_knob #%d - val=%f\n", i, val);
        if (luaRunOnKnob(i, val, shift, &volts, &trigger)) {
          on_successful_lua_callback(i, volts, trigger);
        }
      }
    }
  }
}

uint8_t gammaCorrectUint8_t(float value) {
  return roundf(255.0f * powf(value, 0.8f));
}

const uint8_t const_colors[11][3] = {
    {160, 160, 160},  // White
    {255, 0, 0},      // Red
    {255, 0, 0},      // Red (gate)
    {255, 74, 0},     // Orange
    {250, 175, 0},    // Yellow
    {173, 255, 47},   // Yellow-green
    {0, 255, 0},      // Green
    {0, 255, 255},    // Cyan
    {0, 0, 244},      // Blue
    {200, 0, 255},    // Indigo
    {97, 0, 97},      // Violet
};

void timer_callback_beat(bool on, int user_data) {
  Config *config = &yocto.config[yocto.i][user_data];
  Out *out = &yocto.out[user_data];
  if (config->mode == MODE_CLOCK) {
    if (on && !out->clock_disabled) {
      // trigger the clock
      out->voltage_current = config->max_voltage;
    } else {
      out->voltage_current = config->min_voltage;
    }
  } else if (config->mode == MODE_CODE) {
    float volts;
    bool trigger;
    if (luaRunOnBeat(user_data, on, &volts, &trigger)) {
      on_successful_lua_callback(user_data, volts, trigger);
    }
  }
}

void timer_callback_ws2812(bool on, int user_data) {
  for (uint8_t i = 0; i < 8; i++) {
    // uint8_t jj =
    //     roundf(linlin(yocto.out[i].voltage_current, -5.0, 10.0, 0, 7.4));
    // for (uint8_t j = 0; j < 8; j++) {
    //   WS2812_fill(&ws2812, j, const_colors[jj][0], const_colors[jj][1],
    //               const_colors[jj][2]);
    // }
    // WS2812_show(&ws2812);
    // return;
    // float x = linlin(yocto.out[i].voltage_current, -5.0, 10.0, 0.0, 1.0);
    // uint8_t r, g, b;
    // RGB_Spectra_ToUint8(x, &r, &g, &b);
    // WS2812_fill(&ws2812, i, r, g, b);
    // if (i == 0) {
    //   printf("RGB: %d %d %d\n", r, g, b);
    // }
    // for (uint8_t j = 1; j < 8; j++) {
    //   WS2812_fill(&ws2812, j, r, g, b);
    // }
    // WS2812_show(&ws2812);
    // return;
    if (yocto.out[i].voltage_current < 0) {
      // 0 to -5V goes 0 -> blue with gamma correction
      float t = linlin(yocto.out[i].voltage_current, -5.0, 0.0, 0.0, 1.0);
      uint8_t blue = gammaCorrectUint8_t(t);  // Apply gamma to red intensity
      WS2812_fill(&ws2812, i, 0, 0, 255 - blue);
    } else if (yocto.out[i].voltage_current == 0) {
      // Voltage at 0V means off
      WS2812_fill(&ws2812, i, 0, 0, 0);
    } else if (yocto.out[i].voltage_current <= 5.0) {
      // 0 to 5V goes 0 -> green with gamma correction
      float t = linlin(yocto.out[i].voltage_current, 0.0, 5.0, 0.0, 1.0);
      uint8_t green = gammaCorrectUint8_t(t);  // Apply gamma to green intensity
      WS2812_fill(&ws2812, i, 0, green, 0);
    } else {
      // 5 to 10V goes green -> red with constant perceived brightness
      uint8_t brightness = 255;  // Maximum brightness level
      float t = linlin(yocto.out[i].voltage_current, 5.0, 10.0, 0.0, 1.0);

      // Calculate raw values for red and green
      float red_raw = t;
      float green_raw = 1.0 - t;

      // Apply gamma correction
      uint8_t red = gammaCorrectUint8_t(red_raw);
      uint8_t green = gammaCorrectUint8_t(green_raw);

      // Normalize to constant brightness
      float total = red + green;
      red = roundf((red / total) * brightness);
      green = roundf((green / total) * brightness);

      WS2812_fill(&ws2812, i, red, green,
                  0);  // Set LED with gamma-corrected brightness
    }
  }
  const int leds_second_8[8] = {0, 1, 2, 4, 3, 7, 6, 5};
  for (uint8_t i = 8; i < 16; i++) {
    Config *config = &yocto.config[yocto.i][i - 8];
    Out *out = &yocto.out[i - 8];
    if (out->tuning && blink_on) {
      WS2812_fill(&ws2812, leds_second_8[i - 8] + 8, 0, 0, 0);
    } else {
      uint8_t brightness = 80;
      WS2812_fill(&ws2812, leds_second_8[i - 8] + 8,
                  const_colors[config->mode][0] * brightness / 100,
                  const_colors[config->mode][1] * brightness / 100,
                  const_colors[config->mode][2] * brightness / 100);
    }
  }
  WS2812_show(&ws2812);
}

void timer_callback_check_memory_usage(bool on, int user_data) {
  uint32_t free_heap = getFreeHeap();
  printf_sysex("free_heap %d\n", free_heap);
  if (free_heap < 161216) {
    uint64_t ct = time_us_64();
    luaGarbageCollect();
    printf("GC %d %lld\n", free_heap, time_us_64() - ct);
  }
}

void timer_callback_update_voltage(bool on, int user_data) {
  // update the DAC
  for (uint8_t i = 0; i < 8; i++) {
    DAC_set_voltage(&dac, i, yocto.out[i].voltage_current);
  }
  DAC_update(&dac);
}

void timer_callback_blink(bool on, int user_data) { blink_on = on; }

#ifdef INCLUDE_MIDI
void midi_note_off(int channel, int note) {
  uint32_t ct = to_ms_since_boot(get_absolute_time());
  channel++;  // 1-indexed
#ifdef DEBUG_MIDI
  printf("ch=%d note_off=%d\n", channel, note);
#endif
  bool outs_with_note_change[8] = {false, false, false, false,
                                   false, false, false, false};
  // check if any outputs are set to midi pitch
  for (uint8_t i = 0; i < 8; i++) {
    Config *config = &yocto.config[yocto.i][i];
    Out *out = &yocto.out[i];
    if (config->mode == MODE_NOTE &&
        (config->midi_channel == channel || config->midi_channel == 0) &&
        out->note_on.note == note) {
      // set the voltage
      out->note_on.note = 0;
      out->note_on.time_on = 0;
      outs_with_note_change[i] = true;
      printf("[out%d] note_off %d\n", i + 1, note);
    } else if (config->mode == MODE_CODE) {
      float volts;
      bool trigger;
      bool shift = button_values[8];
      printf("Lua on_note_off #%d - ch=%d, note=%d\n", i, channel, note);
      if (luaRunOnNoteOff(i, channel, note, &volts, &trigger)) {
        // on_successful_lua_callback(i, volts, trigger);
        out->voltage_set = volts;
        outs_with_note_change[i] = !trigger;
      }
    }
  }
  // find any linked outputs and activate the envelope
  update_linked_outs(outs_with_note_change, false, ct);
}

void midi_note_on(int channel, int note, int velocity) {
  uint32_t ct = to_ms_since_boot(get_absolute_time());
  channel++;  // 1-indexed
#ifdef DEBUG_MIDI
  printf("ch=%d note_on=%d vel=%d\n", channel, note, velocity);
#endif
  // special commands
  // 9F 01 01
  if (channel == 16 && note == 1 && velocity == 1) {
    char sparkline_update[48];
    sparkline_update[0] = '\0';
    for (uint8_t i = 0; i < 8; i++) {
      if (i == 0) {
        sprintf(sparkline_update, "%d",
                (int)roundf(linlin(yocto.out[i].voltage_current, -5.0f, 10.0f,
                                   0.0f, 9990.0f)));
      } else {
        sprintf(sparkline_update, "%s_%d", sparkline_update,
                (int)roundf(linlin(yocto.out[i].voltage_current, -5.0f, 10.0f,
                                   0.0f, 9999.0f)));
      }
    }
    // add the current bpm
    sprintf(sparkline_update, "%s_%d", sparkline_update,
            (int)yocto.global_tempo);
    printf_sysex("%s\n", sparkline_update);
    return;
  }

  // check if any outputs are set to midi pitch
  bool outs_with_note_change[8] = {false, false, false, false,
                                   false, false, false, false};
  for (uint8_t i = 0; i < 8; i++) {
    Config *config = &yocto.config[yocto.i][i];
    Out *out = &yocto.out[i];
    if (config->mode == MODE_NOTE) {
      // check if button is pressed
      if (button_values[i]) {
        // learn the channel
        yocto.config[yocto.i][i].midi_channel = channel;
        // save the config
        Yoctocore_schedule_save(&yocto);
      }
      if ((config->midi_channel == channel || config->midi_channel == 0) &&
          (out->note_on.time_on == 0 ||
           (ct - out->note_on.time_on) > MAX_NOTE_HOLD_TIME_MS) &&
          random_integer_in_range(0, 99) < config->probability) {
        // set the voltage for the pitch
        out->note_on.note = note;
        out->note_on.time_on = ct;
        out->voltage_set =
            (float)(note - config->root_note) * config->v_oct / 12.0f +
            config->min_voltage;
        outs_with_note_change[i] = true;
#ifdef DEBUG_MIDI
        printf("[out%d] %d %d %f %f to %f\n", i + 1, note, config->root_note,
               config->v_oct, config->min_voltage, out->voltage_set);
#endif
        break;  // TODO make this an option
      }
    } else if (config->mode == MODE_CODE) {
      float volts;
      bool trigger;
      bool shift = button_values[8];
      printf("Lua on_note_on #%d - ch=%d, note=%d, vel=%d\n", i, channel, note, velocity);
      if (luaRunOnNoteOn(i, channel, note, velocity, &volts, &trigger)) {
        // on_successful_lua_callback(i, volts, trigger);
        out->voltage_set = volts;
        outs_with_note_change[i] = trigger;
      }
    }
  }
  // find any linked outputs and activate the envelope
  update_linked_outs(outs_with_note_change, true, ct);
}

void midi_cc(int channel, int cc, int value) {
  channel++;  // 1-indexed
  printf("ch=%d cc=%d val=%d (%d)\n", channel, cc, value, button_values[0]);
  for (uint8_t i = 0; i < 8; i++) {
    Config *config = &yocto.config[yocto.i][i];
    Out *out = &yocto.out[i];
    if (config->mode == MODE_CONTROL_CHANGE) {
      if (config->midi_channel == channel && config->midi_cc == cc) {
        // set the voltage
        out->voltage_set =
          linlin(value, 0, 127, config->min_voltage, config->max_voltage);
        printf("[cc%d] %f\n", i + 1, out->voltage_current);
      } else if (button_values[i]) {
        // listen and learn the channel and cc
        yocto.config[yocto.i][i].midi_channel = channel;
        yocto.config[yocto.i][i].midi_cc = cc;
        // save the config
        Yoctocore_schedule_save(&yocto);
      }
    } else if (config->mode == MODE_CODE) {
      float volts;
      bool trigger;
      bool shift = button_values[8];
      printf("Lua on_cc #%d - cc=%d, cal=%d\n", i, cc, value);
      if (luaRunOnCc(i, cc, value, &volts, &trigger)) {
        on_successful_lua_callback(i, volts, trigger);
      }
    }
  }
}

void midi_key_pressure(int channel, int note, int pressure) {
  channel++;  // 1-indexed
  printf("ch=%d note=%d pressure=%d\n", channel, note, pressure);
  for (uint8_t i = 0; i < 8; i++) {
    Config *config = &yocto.config[yocto.i][i];
    Out *out = &yocto.out[i];
    if (config->mode == MODE_KEY_PRESSURE) {
      if (config->midi_channel == channel) {
        // set the voltage
        out->voltage_set =
            linlin(pressure, 0, 127, config->min_voltage, config->max_voltage);
        printf("[kp%d] %f\n", i + 1, out->voltage_current);
      } else if (button_values[i]) {
        // listen and learn the channel
        yocto.config[yocto.i][i].midi_channel = channel;
        // save the config
        Yoctocore_schedule_save(&yocto);
      }
    }
  }
}

void midi_program_change(int channel, int program) {
  channel++;  // 1-indexed
  printf("ch=%d program=%d\n", channel, program);
  for (uint8_t i = 0; i < 8; i++) {
    Config *config = &yocto.config[yocto.i][i];
    Out *out = &yocto.out[i];
    if (config->mode == MODE_PROGRAM_CHANGE) {
      if (config->midi_channel == channel) {
        // set the voltage
        out->voltage_set =
            linlin(program, 0, 127, config->min_voltage, config->max_voltage);
        printf("[pc%d] %f\n", i + 1, out->voltage_current);
      } else if (button_values[i]) {
        // listen and learn the channel
        yocto.config[yocto.i][i].midi_channel = channel;
        // save the config
        Yoctocore_schedule_save(&yocto);
      }
    }
  }
}

void midi_channel_pressure(int channel, int pressure) {
  channel++;  // 1-indexed
  printf("ch=%d pressure=%d\n", channel, pressure);
  for (uint8_t i = 0; i < 8; i++) {
    Config *config = &yocto.config[yocto.i][i];
    Out *out = &yocto.out[i];
    if (config->mode == MODE_CHANNEL_PRESSURE) {
      if (config->midi_channel == channel) {
        // set the voltage
        out->voltage_set =
            linlin(pressure, 0, 127, config->min_voltage, config->max_voltage);
        printf("[cp%d] %f\n", i + 1, out->voltage_current);
      } else if (button_values[i]) {
        // listen and learn the channel
        yocto.config[yocto.i][i].midi_channel = channel;
        // save the config
        Yoctocore_schedule_save(&yocto);
      }
    }
  }
}

void midi_pitch_bend(int channel, int value) {
  channel++;  // 1-indexed
  printf("ch=%d pitch_bend=%d\n", channel, value);
  // value is 14-bit number
  for (uint8_t i = 0; i < 8; i++) {
    Config *config = &yocto.config[yocto.i][i];
    Out *out = &yocto.out[i];
    if (config->mode == MODE_PITCH_BEND) {
      if (config->midi_channel == channel) {
        // set the voltage
        out->voltage_set =
            linlin(value, 0, 16383, config->min_voltage, config->max_voltage);
        printf("[pb%d] %f\n", i + 1, out->voltage_current);
      } else if (button_values[i]) {
        // listen and learn the channel
        yocto.config[yocto.i][i].midi_channel = channel;
        // save the config
        Yoctocore_schedule_save(&yocto);
      }
    }
  }
}

#define MIDI_DELTA_COUNT_MAX 24
uint32_t midi_timing_count = 0;
uint64_t midi_last_time = 0;
int64_t midi_timing_differences[MIDI_DELTA_COUNT_MAX] = {0};
void midi_timing() {
  if (midi_last_time == 0) {
    midi_last_time = time_us_64();
    return;
  }

  midi_timing_count++;
  uint64_t now_time = time_us_64();
  int index = midi_timing_count % MIDI_DELTA_COUNT_MAX;

  // Store the time difference between consecutive MIDI Clock messages
  midi_timing_differences[index] = now_time - midi_last_time;

  // Update the last time to the current time
  midi_last_time = now_time;

  // Only recalculate BPM when the buffer wraps around
  if (index == 0) {
    float total_time_us = 0.0f;

    // Sum all the time differences in the buffer
    for (uint8_t i = 0; i < MIDI_DELTA_COUNT_MAX; i++) {
      total_time_us += midi_timing_differences[i];
    }

    // Calculate the average time per tick
    float average_interval_us = total_time_us / MIDI_DELTA_COUNT_MAX;

    // Calculate BPM
    float bpm = (60000000.0f / average_interval_us) / 24.0f;

    // Round to the nearest 10th
    bpm = roundf(bpm);

    // Print the BPM
    printf("bpm: %2.1f\n", bpm);
    yocto.global_tempo = bpm;
  }
}

void midi_event_note_on(char chan, char data1, char data2) {
  midi_note_on(chan, data1, data2);
}

void midi_event_note_off(char chan, char data1, char data2) {
  midi_note_off(chan, data1);
}

void midi_event_control_change(char chan, char data1, char data2) {
  // printf("midi_event_control_change: %d %d %d\n", chan, data1, data2);
  midi_cc(chan, data1, data2);
}

void midi_event_program_change(char chan, char data1, char data2) {
  // printf("midi_event_program_change: %d %d %d\n", chan, data1, data2);
  midi_program_change(chan, data1);
}

void midi_event_pitch_bend(char chan, char data1, char data2) {
  // printf("midi_event_pitch_bend: %d %d %d\n", chan, data1, data2);
  midi_pitch_bend(chan, data1 + (data2 << 7));
}

void midi_event_channel_pressure(char chan, char data1, char data2) {
  // printf("midi_event_channel_pressure: %d %d %d\n", chan, data1, data2);
  midi_channel_pressure(chan, data1);
}

void midi_event_key_pressure(char chan, char data1, char data2) {
  // printf("midi_event_key_pressure: %d %d %d\n", chan, data1, data2);
  midi_key_pressure(chan, data1, data2);
}

void midi_event_clock(char chan, char data1, char data2) { midi_timing(); }

void midi_event_start(char chan, char data1, char data2) {
  printf("midi_event_start: %d %d %d\n", chan, data1, data2);
  midi_start();
}

void midi_event_continue(char chan, char data1, char data2) {
  printf("midi_event_continue: %d %d %d\n", chan, data1, data2);
  midi_continue();
}

void midi_event_stop(char chan, char data1, char data2) {
  printf("midi_event_stop: %d %d %d\n", chan, data1, data2);
  midi_stop();
}
#endif

int main() {
  // Set PLL_USB 96MHz
  const uint32_t main_line = 96;
  pll_init(pll_usb, 1, main_line * 16 * MHZ, 4, 4);
  clock_configure(clk_usb, 0, CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                  main_line * MHZ, main_line / 2 * MHZ);
  // Change clk_sys to be 96MHz.
  clock_configure(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                  CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                  main_line * MHZ, main_line * MHZ);
  // CLK peri is clocked from clk_sys so need to change clk_peri's freq
  clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                  main_line * MHZ, main_line * MHZ);
  // Reinit uart now that clk_peri has changed
  stdio_init_all();

#ifdef DO_OVERCLOCK
  set_sys_clock_khz(225000, true);
#else
  set_sys_clock_khz(125000, true);
#endif
  sleep_ms(10);

  // DCDC PSM control
  // 0: PFM mode (best efficiency)
  // 1: PWM mode (improved ripple)
  gpio_init(PIN_DCDC_PSM_CTRL);
  gpio_set_dir(PIN_DCDC_PSM_CTRL, GPIO_OUT);
  gpio_put(PIN_DCDC_PSM_CTRL, 1);  // PWM mode for less Audio noise

  // setup i2c
  i2c_init(i2c0, 50 * 1000);
  gpio_set_function(I2C0_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(I2C0_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(I2C0_SDA_PIN);
  gpio_pull_up(I2C0_SCL_PIN);
  i2c_init(i2c1, 50 * 1000);
  gpio_set_function(I2C1_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(I2C1_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(I2C1_SDA_PIN);
  gpio_pull_up(I2C1_SCL_PIN);

#ifdef INCLUDE_MIDI
  // setup midi
  tusb_init();

  // setup libmidi
  midi_init();
  midi_register_event_handler(EVT_CHAN_NOTE_ON, midi_event_note_on);
  midi_register_event_handler(EVT_CHAN_NOTE_OFF, midi_event_note_off);
  midi_register_event_handler(EVT_CHAN_CONTROL_CHANGE,
                              midi_event_control_change);
  midi_register_event_handler(EVT_CHAN_PROGRAM_CHANGE,
                              midi_event_program_change);
  midi_register_event_handler(EVT_CHAN_PITCH_BEND, midi_event_pitch_bend);
  midi_register_event_handler(EVT_CHAN_AFTERTOUCH, midi_event_channel_pressure);
  midi_register_event_handler(EVT_CHAN_POLY_AFTERTOUCH,
                              midi_event_key_pressure);
  midi_register_event_handler(EVT_SYS_REALTIME_TIMING_CLOCK, midi_event_clock);
  midi_register_event_handler(EVT_SYS_REALTIME_SEQ_START, midi_event_start);
  midi_register_event_handler(EVT_SYS_REALTIME_SEQ_CONTINUE,
                              midi_event_continue);
  midi_register_event_handler(EVT_SYS_REALTIME_SEQ_STOP, midi_event_stop);

#endif

  // setup pio for uart
  uint offset = pio_add_program(pio0, &uart_rx_program);
  uart_rx_program_init(pio0, 0, offset, MIDI_RX_PIN, 31250);

  // Implicitly called by disk_initialize,
  // but called here to set up the GPIOs
  // before enabling the card detect interrupt:
  sd_init_driver();

  // initialize random library
  random_initialize();

  // initialize knobs
  for (uint8_t i = 0; i < 8; i++) {
    KnobChange_init(&pool_knobs[i], 3);
  }

  // setup buttons
  for (uint8_t i = 0; i < button_num; i++) {
    gpio_init(button_pins[i]);
    gpio_set_dir(button_pins[i], GPIO_IN);
    gpio_pull_up(button_pins[i]);
  }

  // initialize MCP3208
  MCP3208_init(&mcp3208, spi0, PIN_SPI_CSN, PIN_SPI_CLK, PIN_SPI_RX,
               PIN_SPI_TX);

  // initialize WS2812
  WS2812_init(&ws2812, WS2812_PIN, pio0, WS2812_SM, 16);
  WS2812_set_brightness(&ws2812, 50);
  for (uint8_t i = 0; i < 16; i++) {
    WS2812_fill(&ws2812, i, 255, 0, 255);
  }
  WS2812_show(&ws2812);

  // initialize SD card
  // sleep_ms(1000);
  printf("[main]: initializing sd card\n");
  for (uint8_t i = SDCARD_CMD_GPIO - 1; i < SDCARD_D0_GPIO + 5; i++) {
    gpio_pull_up(i);
  }
  if (!run_mount()) {
    sleep_ms(1000);
    printf("[main]: failed to mount sd card\n");
  } else {
    // big_file_test("test.bin", 2, 0);  // perform read/write test
  }

  // initialize lua
  luaInit();
  // sleep_ms(1000);
  // print_memory_usage();

  // initialize the yoctocore
  Yoctocore_init(&yocto);

  uint64_t start_time;
  // load the data
  start_time = time_us_64();
  if (Yoctocore_load(&yocto)) {
    printf("loaded data in %lld us\n", time_us_64() - start_time);
  } else {
    printf("failed to load data\n");
  }

  // // try loading the code for scene 0, output 0
  // Yoctocore_load_code(&yocto, 0, 0);

  // initialize dac
  DAC_init(&dac);
#ifdef DEBUG_VOLTAGE_CALIBRATION
  sleep_ms(5000);
  printf("DAC calibration\n");
  for (int8_t volts = -5; volts <= 10; volts++) {
    for (uint8_t i = 0; i < 8; i++) {
      DAC_set_voltage(&dac, i, volts);
    }
    DAC_update(&dac);
    printf("calibration at %d\n", volts);
    sleep_ms(5000);
  }
#endif
  Yoctocore_get_calibrations(&yocto);
  for (uint8_t i = 0; i < 8; i++) {
    dac.voltage_calibration_slope[i] = yocto.out[i].voltage_calibration_slope;
    dac.voltage_calibration_intercept[i] =
        yocto.out[i].voltage_calibration_intercept;
  }

  for (uint8_t i = 0; i < 8; i++) {
    DAC_set_voltage(&dac, i, 0);
  }
  DAC_update(&dac);

  // initialize timers
  uint32_t ct = to_ms_since_boot(get_absolute_time());
  // first 8 timers are for each output and disabled by default
  for (uint8_t i = 0; i < 16; i++) {
    if (i < 8) {
      SimpleTimer_init(&pool_timer[i], 60.0f, 1.0f, 0, timer_callback_beat, i,
                       ct);
    } else {
      SimpleTimer_init(&pool_timer[i], 60.0f, 1.0f, 0, NULL, i, ct);
    }
  }
  // setup a timer at 5 milliseconds to sample the knobs
  SimpleTimer_init(&pool_timer[8], 1000.0f / 11.0f * 30, 1.0f, 0,
                   timer_callback_sample_knob, 0, ct);
  SimpleTimer_start(&pool_timer[8]);
  // setup a timer at 33 hz to update the ws2812
  SimpleTimer_init(&pool_timer[9], 1000.0f / 30.0f * 30.0f, 1.0f, 0,
                   timer_callback_ws2812, 0, ct);
  SimpleTimer_start(&pool_timer[9]);
  // setup a timer at 3 second to print memory usage
  SimpleTimer_init(&pool_timer[10], 1000.0f / 3000.0f * 30, 1.0f, 0,
                   timer_callback_check_memory_usage, 0, ct);
  SimpleTimer_start(&pool_timer[10]);
  // setup a timer at 4 ms intervals to update voltages
  SimpleTimer_init(&pool_timer[11], 1000.0f / 4.0f * 30, 1.0f, 0,
                   timer_callback_update_voltage, 0, ct);
  SimpleTimer_start(&pool_timer[11]);
  // blinking timer
  SimpleTimer_init(&pool_timer[13], 1000.0f / 370.0f * 30, 1.0f, 0,
                   timer_callback_blink, 0, ct);
  SimpleTimer_start(&pool_timer[13]);

  uint32_t ct_last = ct;
  for (uint8_t i = 0; i < 8; i++) {
    lfo_ct_last[i] = ct;
  }

  printf("Starting main loop\n");

  uint32_t time_last_midi = ct;
  bool button_shift = false;

  // runlua();
  // print_memory_usage();
  // sleep_ms(1000);
  // runlua();
  // print_memory_usage();
  // sleep_ms(1000);
  uint32_t start_time_us = time_us_32();

  while (true) {
    time_per_iteration = time_us_32() - start_time_us;
    start_time_us = time_us_32();
    uint32_t us = time_us_32();
#ifdef INCLUDE_MIDI
    tud_task();
    midi_comm_task(midi_sysex_callback, midi_note_on, midi_note_off,
                   midi_key_pressure, midi_cc, midi_program_change,
                   midi_channel_pressure, midi_pitch_bend, midi_start,
                   midi_continue, midi_stop, midi_timing);
#endif
    timer_per[0] = time_us_32() - us;

    us = time_us_32();
    if (!pio_sm_is_rx_fifo_empty(pio0, 0)) {
      uint8_t ch = uart_rx_program_getc(pio0, 0);
      midi_receive_byte(ch);
    }
    timer_per[1] = time_us_32() - us;

#if CFG_TUD_CDC
    // process serial input
    if (tud_cdc_connected()) {
        int count = tud_cdc_read(cdc_rx_buffer, CFG_TUD_CDC_RX_BUFSIZE);
        if (count) {
            // printf("Received: %s\n", cdc_rx_buffer);
        }
    }
#endif

    // process any mode change
    for (uint8_t i = 0; i < 8; i++) {
      Config *config = &yocto.config[yocto.i][i];
      Out *out = &yocto.out[i];
      if (config->mode != out->mode_last) {
        out->mode_last = config->mode;
        switch (config->mode) {
          case MODE_CODE:
            // load the new code environment
            Yoctocore_load_code(&yocto, yocto.i, i);
            break;
          default:
            break;
        }
      } else {
        // special case
        if (config->mode == MODE_CODE) {
          // check if the code has changed
          if (out->code_updated) {
            // load the new code
            Yoctocore_load_code(&yocto, yocto.i, i);
            // reset the flag
            out->code_updated = false;
          }
        }
      }
    }

    // process timers
    us = time_us_32();
    for (uint8_t i = 0; i < 16; i++) {
      us = time_us_32();
      SimpleTimer_process(&pool_timer[i], ct);
      timer_per[3 + i] = time_us_32() - us;
    }
    timer_per[2] = time_us_32() - us;

    // read buttons
    for (uint8_t i = 0; i < button_num; i++) {
      bool val = 1 - gpio_get(button_pins[i]);
      if (val != button_values[i]) {
        uint8_t unique_id;
        flash_get_unique_id(&unique_id);
        printf("Button %d: %d (%d)\n", i, val, unique_id);
        button_values[i] = val;
        if (i < 8) {
          // process button press
          Out *out = &yocto.out[i];
          Config *config = &yocto.config[yocto.i][i];
          // check mode
          switch (config->mode) {
            case MODE_ENVELOPE:
              // trigger the envelope
              ADSR_gate(&out->adsr, val, ct);
              break;
            case MODE_GATE:
              // set the voltage
              out->voltage_set =
                  val ? config->max_voltage : config->min_voltage;
              break;
            case MODE_NOTE:
              if (button_shift && val) {
                // toggle tuning mode
                out->tuning = !out->tuning;
                printf("[out%d] tuning %d\n", i + 1, out->tuning);
              }
              break;
            case MODE_CLOCK:
              // start and stop the clock
              if (button_shift && val) {
                // tap tempo
                uint16_t bpm_tempo = TapTempo_tap(&out->taptempo);
                if (bpm_tempo > 30 && bpm_tempo < 300) {
                  config->clock_tempo = bpm_tempo;
                  printf("[out%d] tap tempo %d\n", i + 1, bpm_tempo);
                  Yoctocore_schedule_save(&yocto);
                }
              } else if (val) {
                // start/stop
                out->clock_disabled = !out->clock_disabled;
              }
              break;
            case MODE_LFO:
              if (button_shift && val) {
                // toggle lfo
                out->lfo_disabled = !out->lfo_disabled;
              }
              break;
            case MODE_CODE:
              float volts;
              bool trigger;
              bool shift = button_values[8];
              printf("Lua on_button #%d - val=%d\n", i, val);
              if (luaRunOnButton(i, val, shift, &volts, &trigger)) {
                on_successful_lua_callback(i, volts, trigger);
              }
              break;
            default:
              break;
          }
        } else {
          button_shift = val;
        }
      }
    }

    // make sure the rest of the loop doesn't run faster than 500 hz
    ct = to_ms_since_boot(get_absolute_time());
    if (ct - ct_last < 2) {
      continue;
    }
    ct_last = ct;

    // if getting data, skip this for now
    if (ct - yocto.yoctocore_getting < 10) {
      continue;
    }

    // yoctocore save (if debounced)
    start_time = time_us_64();
    if (Yoctocore_save(&yocto, ct)) {
      printf("saved data in %lld us\n", time_us_64() - start_time);
    }

    // process outputs
    for (uint8_t i = 0; i < 8; i++) {
      Out *out = &yocto.out[i];
      Config *config = &yocto.config[yocto.i][i];
      float knob_val = (float)KnobChange_get(&pool_knobs[i]);
      bool button_val = button_values[i];
      // check mode
      // make sure modes are up to date
      if (config->mode == MODE_CLOCK || config->mode == MODE_CODE) {
        SimpleTimer_start(&pool_timer[i]);
        // check bpm
        if (config->mode == MODE_CODE) {
          // get bpm
          float code_bpm = luaGetBPM(i);
          if (code_bpm > 30 && code_bpm < 300) {
            config->clock_tempo = code_bpm;
          }
        }
        if (config->clock_tempo > 0) {
          SimpleTimer_update_bpm(&pool_timer[i], config->clock_tempo,
                                 division_values[config->clock_division]);
        } else {
          // set to global tempo
          SimpleTimer_update_bpm(&pool_timer[i], yocto.global_tempo,
                                 division_values[config->clock_division]);
        }
      } else {
        SimpleTimer_stop(&pool_timer[i]);
      }
      // update slews
      Slew_set_duration(&out->portamento, roundf(config->portamento * 1000));
      Slew_set_duration(&out->slew, roundf(config->slew_time * 1000));

      switch (config->mode) {
        case MODE_LFO:
          // mode lfo will set the voltage based on lfo
          float old_period = config->lfo_period;

          if (knob_val != -1) {
            if (button_shift && !button_val) {
              // set the max voltage
              config->max_voltage =
                  linlin(knob_val, 0.0f, 1023.0f, config->min_voltage, 10.0f);
            } else if (!button_shift && button_val) {
              // set the lfo shape
              config->lfo_waveform = (uint8_t)linlin(
                  knob_val, 0.0f, 1023.0f, 0.0f, (float)LFO_SHAPE_MAX);

            } else {
              if (knob_val < 512) {
                config->lfo_period =
                    10.0f / linexp(knob_val, 0.001f, 512, 0.0333f, 10.0f);
              } else {
                config->lfo_period =
                    10.0f / linexp(knob_val, 512, 1023.0f, 10.0f, 1000.f);
              }
            }
            // printf("changed lfo %d period: %f -> %f\n", i, old_period,
            // config->lfo_period);
          }

          // NB: `step` is basically how much % of the period got elapsed
          uint32_t elapsed_ms = ct - lfo_ct_last[i];
          lfo_ct_last[i] = ct;
          float step = elapsed_ms / (old_period * 1000.f);
          lfo_index_acc[i] = fmod(lfo_index_acc[i] + step, 1.f);

          // if (button_val) {
          //   printf("elapsed=%d ms, p=%f ms, pct=%f\n", elapsed_ms,
          //   (config->lfo_period * 1000.f) , step);
          // }
          float voltage =
              get_lfo_value(config->lfo_waveform, lfo_index_acc[i] * 1000,
                            1 * 1000, config->min_voltage, config->max_voltage,
                            0, &out->noise, &out->slew_lfo);

          if (out->lfo_disabled) {
            break;
          }

          out->voltage_set = voltage;

          // quantize
          out->voltage_current =
              scale_quantize_voltage(config->quantization, config->root_note,
                                     config->v_oct, out->voltage_set);
          break;
        case MODE_NOTE:
          // mode pitch will set the voltage based on midi note
          // button + knob will override and set the voltage
          if (knob_val != -1 && button_val) {
            // change the set voltage
            out->voltage_set = linlin(knob_val, 0.0f, 1023.0f,
                                      config->min_voltage, config->max_voltage);
          }
          // slew the voltage
          out->voltage_current = Slew_process(&out->slew, out->voltage_set, ct);
          // quantize the voltage
          out->voltage_current =
              scale_quantize_voltage(config->quantization, config->root_note,
                                     config->v_oct, out->voltage_current);
          // portamento voltage
          out->voltage_current =
              Slew_process(&out->portamento, out->voltage_current, ct);
          if (out->tuning) {
            out->voltage_current = 3.0;
          }
          if (config->note_tuning > 0) {
            out->voltage_current += ((float)config->note_tuning / 1000.0);
          }
          break;
        case MODE_CONTROL_CHANGE:
        case MODE_KEY_PRESSURE:
        case MODE_PROGRAM_CHANGE:
        case MODE_CHANNEL_PRESSURE:
        case MODE_PITCH_BEND:
          // slew the voltage
          out->voltage_current = Slew_process(&out->slew, out->voltage_set, ct);
          // quantize the voltage
          out->voltage_current =
              scale_quantize_voltage(config->quantization, config->root_note,
                                     config->v_oct, out->voltage_current);
          // portamento voltage
          out->voltage_current =
              Slew_process(&out->portamento, out->voltage_current, ct);
          break;
        case MODE_CLOCK:
          if (knob_val != -1 && button_val && !button_shift) {
            // set the tempo
            config->clock_tempo = linlin(knob_val, 0.0f, 1023.0f, 30.0, 300.0);
            Yoctocore_schedule_save(&yocto);
          } else if (knob_val != -1 && !button_val && button_shift) {
            // set the division
            config->clock_division = linlin(knob_val, 0.0f, 1023.0f, 0, 19);
            Yoctocore_schedule_save(&yocto);
          }
          break;
        case MODE_CODE:
          out->voltage_current = out->voltage_set;
          break;
        case MODE_ENVELOPE:
          // mode envelope will trigger the envelope based on button press
          if (knob_val != -1) {
            if (button_shift) {
              // shift + knob will set the sustain
              config->sustain = linlin(knob_val, 0.0f, 1023.0f, 0.0, 1.0);
            } else if (button_val) {
              // button + knob will set the release
              config->release = linlin(knob_val, 0.0f, 1023.0f, 0.0, 10.0);
            } else {
              // knob will set the attack
              config->attack = linlin(knob_val, 0.0f, 1023.0f, 0.0, 10.0);
            }
            Yoctocore_schedule_save(&yocto);
          }
          out->adsr.attack = roundf(config->attack * 1000);
          out->adsr.decay = roundf(config->decay * 1000);
          out->adsr.sustain = config->sustain;
          out->adsr.release = roundf(config->release * 1000);
          out->voltage_set = linlin(ADSR_process(&out->adsr, ct), 0.0f, 1.0f,
                                    config->min_voltage, config->max_voltage);
          out->voltage_current = out->voltage_set;
          break;
        case MODE_GATE:
          if (knob_val != -1) {
            if (button_shift) {
              // shift + knob will set the probability
              config->probability = linlin(knob_val, 0.0f, 1023.0f, 0.0, 100.0);
            }
            Yoctocore_schedule_save(&yocto);
          }

          out->voltage_current = out->voltage_set;
          break;
        default:
          break;
      }

      // clamp voltages
      out->voltage_current = util_clamp(
          out->voltage_current, config->min_voltage, config->max_voltage);
      // check for voltage override
      if (out->voltage_do_override) {
        out->voltage_current = out->voltage_override;
      }
    }
  }
}
