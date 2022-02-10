## 1. Prerequisites

1.1 Setup ROS environment:
On master PC:
```
export ROS_HOSTNAME=<master_name>
export ROS_MASTER_URI=http://<master_name>:11311/
```
On agent PC:
```
export ROS_HOSTNAME=<agent_name>
export ROS_MASTER_URI=http://<master_name>:11311/

```

## 2. Build on ROS

2.1 Download and build package:
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

2.2 launch package on startup:
Copy file "agent_start.service" to "/etc/systemd/system"
```
    cd ~/catkin_ws/src/socket_streamer/sysServices
    sudo cp agent_start.service /etc/systemd/system
    
    # Reload the service files to include the new service.
    sudo systemctl daemon-reload

    # To enable your service on every reboot
    sudo systemctl enable agent_start.service

    # Start your service
    sudo systemctl start your-agent_start.service

    # To check the status of your service
    sudo systemctl status agent_start.service

    # If needed
    sudo reboot
```
## 3. Run with your device
There is several ways to execute:
1. Manually on agent:
```
roslaunch socket_streamer agent_run.launch
roslaunch socket_streamer make_dataset.launch (For recording)
```

2. With SSH on agent (need to configure ssh):
```
ssh <agent_name>
roslaunch socket_streamer agent_run.launch
roslaunch socket_streamer make_dataset.launch (For recording)
```

3. Automatic on startup (check section 2.2)