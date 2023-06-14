#!/bin/bash

## if no parameter, then exit after print usage
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 target"
    echo "target: select one among these"
    echo "jetracer, jetbot, motorhat2wheel, motorhatSteer, nuriBldc, 298n2Wheel"
	exit 1
fi

cp ../jessicar_control/config/motor.$1.yaml  ../jessicar_control/config/motor.yaml
cp ../jessicar_joy/config/joy_teleop.$1.yaml  ../jessicar_joy/config/joy_teleop.yaml