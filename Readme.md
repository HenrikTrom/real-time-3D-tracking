# Real-Time-3D-Tracking 

TODO:

-Color-> yolo confidence 0.3 fps 60
- keypoint -> confidence 60, fps, 50
- Video Logging

- Fix clock in cameras and pass timestamps


This module is part of my  **ROS/ROS2** [real-time 3D tracker docker-implementation](https://github.com/HenrikTrom/ROSTrack-RT-3D).


    // !understand how correspondance and aabb calculate work, notes -> LaTex
    // !# pre-Thesis notes

Usage

1. Calibrate Cameras

2. Calibrate Robot (test)

3. Run interface


Adapt general settings in cfg

Adapt things like topic names and filepaths in `./cfg/config.h.in`

Change REAL Time clock with system clock -> get latency