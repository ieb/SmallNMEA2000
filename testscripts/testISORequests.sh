#!/bin/bash

# This script requires a working socketcan + canboat installation
# I am using candellight CAN-USB adapter on a linux box. Wont work on OSX as it has no support
# for device.

target="${1:-255}"

productInformation=$(php util/format-message ${target} request_pgn 126996) 
pgnList=$(php util/format-message ${target}  request_pgn 126464)
configurationInformation=$(php util/format-message ${target}  request_pgn 126998)
batteryConfigurationInformation=$(php util/format-message ${target}  request_pgn 127513)

echo "Will send reqests to target address ${target}"



candump can0 | candump2analyzer | analyzer &
sleep 2
echo "## Sending ISO Request for Product Information as ${productInformation}"
echo ${productInformation} |  socketcan-writer can0
sleep 5
echo "## Sending ISO Request for PGN List as ${pgnList}"
echo ${pgnList} |  socketcan-writer can0
sleep 5
echo "## Sending ISO Request for Configuration Information as ${configurationInformation}"
echo ${configurationInformation} |  socketcan-writer can0
sleep 5
echo "## Sending ISO Request for Battery Configuration Information as ${batteryConfigurationInformation}"
echo ${batteryConfigurationInformation} |  socketcan-writer can0

sleep 5
kill %1