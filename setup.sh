#!/bin/bash

PIN=79
echo $PIN > "/sys/class/gpio/export"
echo 'out' > "/sys/class/gpio/gpio$PIN/direction"
echo '0' > "/sys/class/gpio/gpio$PIN/value"

echo 1000000 > "/sys/bus/i2c/devices/i2c-1/bus_clk_rate"