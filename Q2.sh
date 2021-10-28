#!/bin/bash  
rostopic pub /cmd_vel geometry_msgs/Twist -r 10 " linear:
        x: 0.22
        y: 0.0
        z: 0.0
 angular:
        x: 0.0
        y: 0.0
        z: 0.1333333"
