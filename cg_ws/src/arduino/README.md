## Set Up Micro-ROS for Arduino Due

First ensure that you install the Due board using the Board Manager within Arduino IDE, instructions can be found in the Quickstart Guide here https://docs.arduino.cc/hardware/due

Download the precompiiled library and add it to Arduino IDE by following these instructions https://github.com/micro-ROS/micro_ros_arduino/tree/galactic#how-to-use-the-precompiled-library

Apply the following patch listed under Patch SAMD: https://github.com/micro-ROS/micro_ros_arduino/tree/galactic#patch-samd. The `ARDUINO_PATH` that they refer to is most likely located at the following:
- On GNU/Linux: ~/.arduino15/packages/
- On Windows: %AppData%\Arduino15\packages\
- On macOS: ~/Library/Arduino15/packages/
More information about where cores are installed: https://support.arduino.cc/hc/en-us/articles/360012076960-Where-are-the-installed-cores-located-

Open Arduino IDE (close and re-open if already open). Go to Tools -> Board -> Arduino ARM and set the board to Arduino Due  (Programming Port)

Open a micro ROS example, e.g. File -> Examples -> micro_ros_arduino -> micro-ros_publisher. Verify it compiles. If you have the Arduino Due with you, you can try to upload it. 

## Set Up Micro-ROS inside Docker Container

Follow the steps here to install micro-ROS: https://micro.ros.org/docs/tutorials/core/first_application_linux/

Do everything from cloning the micro_ros_setup repo, up to and including "Creating the micro-ROS agent" section, with a few modifications

Specifically, replicate these steps:
* `cd microros_ws`
* `git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup`
* `sudo apt update && rosdep update`
* `rosdep install --from-path src --ignore-src -y`
* (Skipped over the pip installation step)
* `colcon build`
* `source install/local_setup.zsh`
* `ros2 run micro_ros_setup create_firmware_ws.sh host`
* `ros2 run micro_ros_setup build_firmware.sh`
* `source install/local_setup.zsh`
* `ros2 run micro_ros_setup create_agent_ws.sh`
* `ros2 run micro_ros_setup build_agent.sh`
* `source install/local_setup.zsh`

If Docker container is brought down and restarted, need to rerun steps from `source install/local_setup.zsh` and `ros2 run micro_ros_setup create_agent_ws.sh`

## Testing Communication with Due

Verify that the Due is readable from within the Docker container, e.g it exists at `/dev/cg_dev/tty_aruino` inside the container.

Upload the micro-ros_publisher example to the Due.

In the Docker container, run `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/cg_dev/tty_arduino -v6`. You should start to see several `send_message` and `recv_message` debug messages printed to console.

Run `ros2 topic list`. You should be able to see the topic `/micro_ros_arduino_node_publisher`. Echo the topic and verify that data is being received from the Due.

NOTE: It appears that if the `micro_ros_agent` is killed and restarted, messages won't appear. In this case, if you don't see the topic, you may need to hit the RESET button on the Arduino

## Reflashing the Due

To upload a new sketch the Arduino, make sure that `micro_ros_agent` is not running (otherwise it will hog the serial interface, and you'll end up with an error in the Arduino IDE, something like `SAM-BA operation failed`).

After the sketch is uploaded, restart `micro_ros_agent`. You will also probably have to hit the RESET button on Arduino as well (seems like `micro_ros_agent` needs to run first before Arduino).
