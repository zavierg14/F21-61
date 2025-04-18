#!/bin/bash

sudo ip link set can0 up type can bitrate 500000

sleep 1

source /home/admin/env/bin/activate

python3 /home/admin/F21-61/Integration/pi2/IntegrationPi2.py
