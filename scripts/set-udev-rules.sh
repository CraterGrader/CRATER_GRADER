#Remove udev rules file if it's already there for clean build
sudo rm /etc/udev/rules.d/49-cg-custom.rules

# Instantiate the new udev rules file if it doesn't exist
sudo touch /etc/udev/rules.d/49-cg-custom.rules

# Arduino
sudo echo 'KERNEL=="ttyACM[0-9]*", SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="003d", SYMLINK="cg_dev/tty_arduino"' | sudo tee /etc/udev/rules.d/49-cg-custom.rules -a

# DWM1001-Dev Beacon
sudo echo 'KERNEL=="ttyACM[0-9]*", SUBSYSTEM=="tty", ATTRS{idVendor}=="1366", ATTRS{idProduct}=="0105", SYMLINK="cg_dev/tty_uwb1"' | sudo tee /etc/udev/rules.d/49-cg-custom.rules -a

# IMU 
sudo echo 'KERNEL=="ttyUSB[0-9]*", SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK="cg_dev/tty_imu"' | sudo tee /etc/udev/rules.d/49-cg-custom.rules -a

# IMU 
sudo echo 'KERNEL=="video[0-9]*", SUBSYSTEM=="video4linux", ATTRS{idVendor}=="32e4", ATTRS{idProduct}=="9230", SYMLINK="cg_dev/sun_cam"' | sudo tee /etc/udev/rules.d/49-cg-custom.rules -a

# Reset the udev rules immediately, run the following command:
sudo udevadm control --reload-rules && udevadm trigger