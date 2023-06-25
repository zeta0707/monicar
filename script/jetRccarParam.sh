#!/bin/bash

## if no parameter, then exit after print usage
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 target"
    echo "target: select one among these"
    echo "jetracer, jetbot, motorhat2wheel, motorhatSteer, nuriBldc, 298n2Wheel, pca9685Steer"
	exit 1
fi

cp ../monicar_control/param/motor.$1.yaml  ../monicar_control/param/motor.yaml
cp  ../monicar_control/param/motor.yaml  ../monicar_control/param/motor.4blob.yaml
sed -i 's/motor_control_node/blob_chase_node/g' ../monicar_control/param/motor.4blob.yaml
cp  ../monicar_control/param/motor.yaml  ../monicar_control/param/motor.4chase.yaml
sed -i 's/motor_control_node/chase_ball_node/g' ../monicar_control/param/motor.4chase.yaml