#!/bin/bash -x   
echo Initializing SETSERIAL LOW_LATENCY
sudo chmod a+rw /dev/ttyUSB0
sudo setserial /dev/ttyUSB0 low_latency
