# Controlling kawasaki fs20n robotic arm via ROS by MoveIt!

Usage with robot emulator:

1. In config file `connection.yaml` in the `kawasaki_fs20n_control/config` folder set following settings:

```
robot_ip: "127.0.0.1"
robot_port: 5555
```

2. Run `robot_server_emulator`:

```
cd [location_of_ws]/kawasaki_fs20n_control_ws/src/kawasaki_fs20n_control
./robot_server_emulator
```

3. Launch `kawasaki_fs20n_control.launch`:

```
roslaunch kawasaki_fs20n_control kawasaki_fs20n_control.launch
```

4. In RViz window you can use MoveIt! plugin to plan manipulator trajectory and execute it.


Usage with real robot:

1. Launch `kawasaki_fs20n_control.launch`:

```
roslaunch kawasaki_fs20n_control kawasaki_fs20n_control.launch
```

2. In RViz window you can use MoveIt! plugin to plan manipulator trajectory and execute it.