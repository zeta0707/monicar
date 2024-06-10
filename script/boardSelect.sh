#!/bin/bash

## if no parameter, then exit after print usage
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 target"
    echo "target: select one between these"
    echo "orin, nano"
	exit 1
fi

if [ "$1" == "orin" ]; then
    sed -i 's/IIC_BUS = .*/IIC_BUS = 7/g' ../monicar_control/monicar_control/motor_control.py
    sed -i 's/IIC_BUS = .*/IIC_BUS = 7/g' ../monicar_control/monicar_control/blob_chase.py
else
    sed -i 's/IIC_BUS = .*/IIC_BUS = 1/g' ../monicar_control/monicar_control/motor_control.py
    sed -i 's/IIC_BUS = .*/IIC_BUS = 1/g' ../monicar_control/monicar_control/blob_chase.py
fi
