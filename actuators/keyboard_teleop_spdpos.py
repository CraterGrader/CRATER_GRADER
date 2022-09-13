from sshkeyboard import listen_keyboard, stop_listening
import serial
import time
import argparse


class KeyboardTeleop():
  def __init__(self, serial_port='COM6', serial_baudrate=115200):
    self.zero_spd = 0
    self.zero_pos = 0
    self.estop_val = 0  # 0 := run, 1 := stop
    # Board1 M1 = Rear Drive
    self.b1m1 = 0
    # Board1 M2 = Rear Steer
    self.b1m2 = 1
    # Board2 M1 = Front Drive
    self.b2m1 = 2
    # Board2 M2 = Front Steer
    self.b2m2 = 3
    # Emergency stop index
    self.estop_idx = 4
    # Byte array for buffer: [Board1 M1, Board1 M2, Board2 M1, Board2 M2, estop]
    self.curr_cmds = [self.zero_spd,
                      self.zero_pos, self.zero_spd, self.zero_pos, self.estop_val]  # Must be integers in [0,255]
    self.spd_increment = 100  # Scaling factor for speed = in [qpps]
    # Scaling factor for position = ~15deg output, by encoder counts and gear ratio
    self.pos_increment = 194
    self.offset = 127  # Where to center resulting values at
    # self.spd_min = 4
    # self.spd_max = 124
    # self.pos_min = 4
    # self.pos_max = 124

    # Establish serial interface
    self.serial_interface = serial.Serial(
        port=serial_port, baudrate=serial_baudrate, timeout=2)
    time.sleep(1)  # wait a couple seconds

    # Initialize the speeds to zero
    self.write_read(self.curr_cmds)

    # Tell user the interface is ready
    print("Keyboard Teleop Ready!")
    print("======================")

    # Start up the keyboard listener
    listen_keyboard(on_press=self.handle_press, delay_second_char=0,
                    delay_other_chars=0, sleep=0)

  def write_read(self, curr_cmds):
    """
    Write to arduino over serial and read the value written.
    Args:
      curr_cmds (list of integers): The array of commands to write to arduino.
    Returns:
      data (str): The value that the arduino read.
    """
    # x = self.clamp(x, self.min_val, self.max_val)  # Clamp to actuator limits
    # Board1 M1 - Speed control
    scaled_b1m1 = self.scale_and_offset(
        curr_cmds[self.b1m1], self.spd_increment, self.offset)
    # Board1 M2 - Position control
    scaled_b1m2 = self.scale_and_offset(
        curr_cmds[self.b1m2], self.pos_increment, self.offset)
    # Board2 M1 - Speed control
    scaled_b2m1 = self.scale_and_offset(
        curr_cmds[self.b2m1], self.spd_increment, self.offset)
    # Board2 M2 - Position control
    scaled_b2m2 = self.scale_and_offset(
        curr_cmds[self.b2m2], self.pos_increment, self.offset)

    # Create byte array for scaled commands
    scaled_cmds = [scaled_b1m1, scaled_b1m2, scaled_b2m1,
                   scaled_b2m2, curr_cmds[self.estop_idx]]

    # Write to serial
    print(f"Writing: {curr_cmds} --> {scaled_cmds} --> {bytes(scaled_cmds)}")
    self.serial_interface.write(
        bytes(scaled_cmds))  # Write scaled commands
    self.curr_cmds = curr_cmds  # Update the current commands

    # Read from serial
    print("Please wait, reading...")
    time.sleep(0.05)  # Brief pause for serial transfer
    # data = "TEST"  # Uncomment to give fast input (skips reading)
    data = self.serial_interface.read(1000)  # Uncomment to actually read data
    print(f"Read: {data}")

    print("--")
    return data

  def scale_and_offset(self, val, scale, offset):
    """
    Scales a value into 0-255 byte range, then centers at offset
    Args:
      val (int): Value to convert to a byte
      scale (int): How much to scale value by, must be a factor of val
      offset (int): Where to center scaled value at, to enable negative numbers
    Returns:
      scaled_val (int): Integer byte on scale of [0,255]
    """
    # Check input
    assert type(val) == int, "val must be an integer"
    assert type(scale) == int, "scale must be an integer"
    assert type(offset) == int, "offset must be an integer"
    assert val % scale == 0, "scale should be a factor of val"

    # Perform the operation
    scaled_val = (val / scale) + offset
    # Clamp to make sure value within byte range
    scaled_val = int(self.clamp(scaled_val, 0, 255))
    return scaled_val

  def clamp(self, val, min_val, max_val):
    """
    Ensures value stays within a range [min_val, max_val]. Does not modify value if the value is within the range bounds.
    Args:
      val (numeric - float/int/etc.): The value to be clamped.
      min_val (numeric - float/int/etc.): The minimum range value.
      max_val (numeric - float/int/etc.): The maximum range value.
    Returns:
      The clamped value, on the range [min_val, max_val]
    """
    return max(min_val, min(val, max_val))

  def clean_and_close(self):
    """
    Close out the teleop interfaces, then exit the program.
    Args:
      None
    Returns:
      None
    """
    stop_listening()  # Stop the keyboard listener
    self.estop_val = 1  # Tell arduino to stop everything
    self.curr_cmds[self.estop_idx] = self.estop_val
    self.write_read(self.curr_cmds)  # Write out zero speed
    self.serial_interface.close()  # Close out the serial interface
    # Quit the program
    try:
      raise SystemExit
    except SystemExit as e:
      # Catch error for output handling, system will still exit
      print("Exiting...")

  def handle_press(self, key):
    """
    Handle a key press with different cases depending on the key.
    Args:
      key (str): The string value of the key that was pressed.
    Returns:
      None
    """
    print(f"Key: {key}")
    if key == 'up':
      # Increase speed for Drive motors
      self.curr_cmds[self.b1m1] += self.spd_increment
      self.curr_cmds[self.b2m1] -= self.spd_increment
      wrote_val = self.write_read(self.curr_cmds)
    elif key == 'down':
      # Decrease speed for Drive motors
      self.curr_cmds[self.b1m1] -= self.spd_increment
      self.curr_cmds[self.b2m1] += self.spd_increment
      wrote_val = self.write_read(self.curr_cmds)

    elif key == 'right':
      # Increase position for Steer motors
      self.curr_cmds[self.b1m2] += self.pos_increment
      self.curr_cmds[self.b2m2] -= self.pos_increment
      wrote_val = self.write_read(self.curr_cmds)
    elif key == 'left':
      # Decrease position for Steer motors
      self.curr_cmds[self.b1m2] -= self.pos_increment
      self.curr_cmds[self.b2m2] += self.pos_increment
      wrote_val = self.write_read(self.curr_cmds)
    elif key == 'space':
      # Stop everything
      self.estop_val = 1  # Tell arduino to stop everything
      self.curr_cmds[self.estop_idx] = self.estop_val
      wrote_val = self.write_read(self.curr_cmds)  # Write out with estop set
    elif key == 'r':
      # Start everything
      self.estop_val = 0  # Tell arduino to start everything
      self.curr_cmds[self.estop_idx] = self.estop_val
      wrote_val = self.write_read(self.curr_cmds)  # Write out with estop set
    elif key == 'q':
      # Quit the program
      self.clean_and_close()


def parse_args():
  parser = argparse.ArgumentParser()
  parser.add_argument(
      '-p', '--port',
      type=str,
      help='Serial port to communicate to',
      default='COM6'
  )
  return parser.parse_args()


if __name__ == '__main__':
  # Set port for serial communication
  # alex_mac_port = '/dev/cu.usbmodem14101'
  # ryan_jupyternb_port = 'COM6'
  args = parse_args()

  # Startup the teleop, give input through terminal
  keyboard_teleop = KeyboardTeleop(serial_port=args.port)
