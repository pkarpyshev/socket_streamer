## 1. Prerequisites



1.1 **Ubuntu** and **ROS**
Clone the repository 
```
    cd ~/catkin_ws/src
    git clone https://github.com/Alaric102/socket_streamer.git
```

and catkin_make:
```
    cd ~/catkin_ws
    catkin_make
    # Recommended on Raspberry Pi
    # catkin_make --only-pkg-with-deps socket_streamer --make-args -j2 --cmake-args -O3
    source ~/catkin_ws/devel/setup.bash
```

Or with Makefile:
```
    cd ~/catkin_ws/src/socket_streamer
    make all
```

## 2. Build on ROS

## 3. Run with your device 
