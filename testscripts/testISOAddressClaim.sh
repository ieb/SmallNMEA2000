#!/bin/bash

# This script requires a working socketcan + canboat installation
# need php cli installed.
# I am using candellight CAN-USB adapter on a linux box. Wont work on OSX as it has no support
# for device.
# $ sudo ip link set can0 type can bitrate 250000
# $ sudo ip link set up can0

target="${1:-255}"

addressClaim=$(php util/format-message ${target} request_pgn 60928)

echo "Will send reqests to target address ${target}"



candump can0 | candump2analyzer | analyzer | grep "60928" &
sleep 2
echo ${addressClaim} |  socketcan-writer can0
sleep 10
echo ${addressClaim} |  socketcan-writer can0
sleep 10
kill %1