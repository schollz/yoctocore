// Copyright 2023-2024 Zack Scholl.
//
// Author: Zack Scholl (zack.scholl@gmail.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// See http://creativecommons.org/licenses/MIT/ for more information.

#ifndef LIB_MCP3208_H
#define LIB_MCP3208_H 1

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "hardware/spi.h"
#include "pico/stdlib.h"

typedef struct MCP3208 {
  uint8_t cs_pin;
  uint8_t sck_pin;
  uint8_t mosi_pin;
  uint8_t miso_pin;
  spi_inst_t *spi;
  uint8_t *buffer;
  uint8_t *data;
} MCP3208;

void MCP3208_init(MCP3208 *self, spi_inst_t *spi, uint8_t cs_pin,
                  uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin) {
  self->cs_pin = cs_pin;
  self->sck_pin = sck_pin;
  self->mosi_pin = mosi_pin;
  self->miso_pin = miso_pin;
  self->spi = spi;
  self->buffer = (uint8_t *)malloc(3 * sizeof(uint8_t));
  self->data = (uint8_t *)malloc(3 * sizeof(uint8_t));

  self->buffer[0] = 0x01;  // Start bit default
  self->buffer[2] = 0x00;  // Dummy byte

  // Initialize CS pin high
  gpio_init(self->cs_pin);
  gpio_set_dir(self->cs_pin, GPIO_OUT);
  gpio_put(self->cs_pin, 1);

  // Initialize SPI port at 0.5 MHz
  spi_init(spi, 1000 * 1000);

  // Initialize SPI pins
  gpio_set_function(self->sck_pin, GPIO_FUNC_SPI);
  gpio_set_function(self->mosi_pin, GPIO_FUNC_SPI);
  gpio_set_function(self->miso_pin, GPIO_FUNC_SPI);
}

uint16_t MCP3208_read(MCP3208 *self, uint8_t channel, bool differential) {
  uint16_t val = 0;

  // Validate channel (0-7 for single-ended, 0-3 for differential)
  if (channel > 7 || (differential && channel > 3)) {
    printf("Invalid channel: %d\n", channel);
    return 0xFFFF;  // Return error code for invalid channel
  }

  // Construct SPI command
  self->buffer[0] = 0x01;  // Start bit
  self->buffer[1] = ((differential ? 0 : 1) << 7) | (channel << 4);
  self->buffer[2] = 0x00;  // Dummy byte

  // Assert CS
  gpio_put(self->cs_pin, 0);

  // Send command and read response
  int num_bytes_wrote =
      spi_write_read_blocking(self->spi, self->buffer, self->data, 3);

  // Deassert CS
  gpio_put(self->cs_pin, 1);

  // Process response if SPI communication succeeded
  if (num_bytes_wrote == 3) {
    val = ((self->data[1] & 0x03) << 8) | self->data[2];
  } else {
    printf("SPI communication failed\n");
    return 0xFFFF;  // Return error code for SPI failure
  }

  return val;
}

#endif