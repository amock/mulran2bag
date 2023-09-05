# mulran2bag

Converts mulran structure to a bag file

## Parameters

Right now you have to change the code.


## Data

### Ouster LiDAR

### Navtech Radar

### GPS

### Xsens IMU



## Transformations

![tf tree](dat/tf_tree.png "tf tree")

## Dynamic Transformations

The ground truth is used to generate a dynamic transformation  

## Static Transformations

All static transformations are generated exactly as a static transform broadcaster would do it and are written to the `tf_static`-topic.

Currently implemented **static** transformations:
- base to ouster
- base to navtech

Currently missing static transformations:
- base to gps (set to identity)
- base to imu (set to identity)

Feel free to contact me if you know them. I set these missing transformations to identity.
