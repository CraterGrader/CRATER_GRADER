from sshkeyboard import listen_keyboard, stop_listening
import serial
import time


class KeyboardTeleop():
  def __init__(self, serial_port='COM6', serial_baudrate=115200):
    self.zero_speed = 64
    self.curr_speed = self.zero_speed
    self.ctrl_increment = 5
    self.min_val = 4  # Can go >= 0, must be <= self.max_val
    self.max_val = 124  # Could go <= 127, must be >= self.min_val

    # Establish serial interface
    self.serial_interface = serial.Serial(
        port=serial_port, baudrate=serial_baudrate, timeout=.1)

    # Initialize the speed to zero
    self.write_read(self.zero_speed)

    # Start up the keyboard listener
    listen_keyboard(on_press=self.handle_press, delay_second_char=0,
                    delay_other_chars=0, sleep=0)

  def write_read(self, x):
    """
    Write to arduino over serial and read the value written.
    Args:
      x (numeric - float/int/etc.): The value to write to arduino.
    Returns:
      data (str): The value that the arduino read.
    """
    x = self.clamp(x, self.min_val, self.max_val)  # Clamp to actuator limits
    self.serial_interface.write(bytes(str(x), 'utf-8'))  # Write data
    self.curr_speed = x  # Update the current speed
    time.sleep(0.05)  # Brief pause for serial transfer
    data = self.serial_interface.readline()  # Read serial
    print(f"Writing: {x}, Read: {data}, curr_speed: {self.curr_speed}")
    print("--")
    return data

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
    self.write_read(self.zero_speed)  # Write out zero speed
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
      # Increase speed
      write_val = self.curr_speed + self.ctrl_increment
      wrote_val = self.write_read(write_val)
    elif key == 'down':
      # Decrease speed
      write_val = self.curr_speed - self.ctrl_increment
      wrote_val = self.write_read(write_val)
    elif key == '0':
      # Decrease speed
      write_val = self.zero_speed
      wrote_val = self.write_read(write_val)
    elif key == 'q':
      # Quit the program
      write_val = self.zero_speed
      wrote_val = self.write_read(write_val)
      self.clean_and_close()


if __name__ == '__main__':
  # Set port for serial communication
  alex_mac_port = '/dev/cu.usbmodem14101'
  ryan_jupyternb_port = 'COM6'

  # Startup the teleop, give input through terminal
  keyboard_teleop = KeyboardTeleop(serial_port=alex_mac_port)
