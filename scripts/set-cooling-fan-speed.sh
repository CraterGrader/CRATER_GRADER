#!/bin/sh

# Set cooling fan to always spin at full speed [0=off, 127= half speed, 255=full speed]
echo 200 | sudo tee /sys/devices/pwm-fan/target_pwm
