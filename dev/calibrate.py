import mido
from digital_multimeter.main import DigitalMultimeter
from icecream import ic
from scipy.stats import linregress
import matplotlib.pyplot as plt

import glob


def list_tty_usb_devices():
    # Look for devices under the standard TTY USB path
    devices = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    return devices


def float_to_14bit(float_value):
    if not (0 <= float_value <= 1):
        raise ValueError("Input float must be in the range 0 to 1.")

    # Scale float to 14-bit range (0 to 16383)
    scaled_value = round(float_value * 16383)

    # Extract the high 7 bits (most significant bits)
    high7_bits = (scaled_value >> 7) & 0x7F

    # Extract the low 7 bits (least significant bits)
    low7_bits = scaled_value & 0x7F

    # Return the two 7-bit numbers
    return high7_bits, low7_bits


def send_voltage(channel, voltage, midi_device):
    """
    Send the scaled voltage as a MIDI SysEx message.

    :param channel: MIDI channel (1-indexed)
    :param scaled_voltage: A float between 0 and 1
    :param midi_device: A MIDI output object (e.g., provided by mido)
    """
    scaled_voltage = (voltage + 5.0) / 15.0
    # Convert the scaled voltage to 14-bit
    high7_bits, low7_bits = float_to_14bit(scaled_voltage)

    # Create the Control Change message
    message = mido.Message(
        "control_change", channel=channel - 1 + 8, control=high7_bits, value=low7_bits
    )

    # Send the message using the provided MIDI device
    ic(channel, voltage, message)
    midi_device.send(message)


def run():
    output = None
    for device in mido.get_output_names():
        if "yoctocore" in device:
            output = mido.open_output(device)
            break

    if output is None:
        print("No yoctocore device found.")
        return

    ic(device)
    # send_voltage(1, 2.12, output)

    # list usb devices using pyusb
    usb_devices = list_tty_usb_devices()
    ic(usb_devices)
    if len(usb_devices) == 0:
        print("No USB devices found.")
        return

    # find multimeters
    multimeter = None
    for device in usb_devices:
        try:
            multimeter = DigitalMultimeter(connect=device)
            ic(multimeter)
            break
        except Exception as e:
            ic(e)

    if multimeter is None:
        print("No multimeter found.")
        return

    # for each of the 8 ouputs, measure 15 voltages
    # make subplots 2x4
    (fig, axs) = plt.subplots(2, 4)
    for channel in range(1, 9):
        voltages = [-4.8, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9.9]
        measured_voltages = []
        for voltage in voltages:
            send_voltage(channel, voltage, output)
            # measure voltage
            voltage = multimeter.measure_voltage()
            ic(channel, voltage)
            measured_voltages.append(voltage)
        # compute regression between measured voltages and expected voltages
        slope, intercept, r_value, p_value, std_err = linregress(
            voltages, measured_voltages
        )
        # plot the points and the regression
        axs[(channel - 1) // 4, (channel - 1) % 4].scatter(voltages, measured_voltages)
        axs[(channel - 1) // 4, (channel - 1) % 4].plot(
            voltages, [slope * x + intercept for x in voltages]
        )
        axs[(channel - 1) // 4, (channel - 1) % 4].set_title(f"Channel {channel}")
        axs[(channel - 1) // 4, (channel - 1) % 4].set_xlabel("Expected voltage (V)")
        axs[(channel - 1) // 4, (channel - 1) % 4].set_ylabel("Measured voltage (V)")
    plt.show()


if __name__ == "__main__":
    run()