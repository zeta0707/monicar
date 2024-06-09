#!/bin/bash

## if no parameter, then exit after print usage
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 target"
    echo "target: select one between these"
    echo "csicam, usbcam"
	exit 1
fi

if [ "$1" == "usbcam" ]; then
    sed -i 's/csicam/usbcam/g' ../monicar_control/launch/blob_all.launch.py
    sed -i 's/csicam/usbcam/g' ../monicar_control/launch/traffic_all.launch.py
    sed -i 's/csicam/usbcam/g' ../monicar_control/launch/yolo_all.launch.py
else
    sed -i 's/usbcam/csicam/g' ../monicar_control/launch/blob_all.launch.py
    sed -i 's/usbcam/csicam/g' ../monicar_control/launch/traffic_all.launch.py
    sed -i 's/usbcam/csicam/g' ../monicar_control/launch/yolo_all.launch.py
fi
