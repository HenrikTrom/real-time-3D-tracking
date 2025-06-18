# Real-Time-3D-Tracking 

TODO:

- Adjust cameras 2 front up, 2 sideways () top->front
- include top camera in triangulation??
- Add latency checks

- cv resize error 

Online Test
- benchmark
- video
- 133
- color
- better cameras (fix aabbs?)
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