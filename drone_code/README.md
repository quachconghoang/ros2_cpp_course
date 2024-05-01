# tutorials
- Start envs 2nd: 
```bash
ros2 launch gazebo_tf a2.launch.py
```
- Drone taking off: 
```bash
ros2 topic pub -1 /drone/takeoff std_msgs/msg/Empty
```
- Call Api
