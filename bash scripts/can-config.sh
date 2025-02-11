#! /bin/bash

apt-get install can-utils
ip link set can0 up type can bitrate 500000
