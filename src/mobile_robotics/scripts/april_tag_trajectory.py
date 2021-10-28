import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PointStamped
from apriltag_ros.msg import AprilTagDetectionArray
import tf2_ros
import tf2_geometry_msgs
import time

import numpy as np
from scipy import linalg


class Assignment_2:

    def __init__(self):
        self.april_tag_detection = None
        self.tfBuffer = tf2_ros.Buffer()   # needs to be defined in constructor otherwise creates issues.
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.check =  True
        self.linear_speed = 0
        self.angular_speed = 0
        self.time = 0
        
    
    def inverse_kinematics(self, final_pose, initial_pose, t):
        
        val = (1.0/t)*linalg.logm(np.matmul(np.linalg.inv(initial_pose), final_pose))
        self.linear_speed = val[0][2]
        self.angular_speed = val[1][0]

        if self.linear_speed > 0.22:
            print("Cannot reach goal within specified time")
            print("linear speed = ", self.linear_speed)
            print("angular speed = ", self.angular_speed)
            print("limiting speed to permissible values, giving maximum controls to turtlebot for specified time")
            self.linear_speed = 0.22
        else:
            print("Giving control commands to turtlebot to reach the goal")
            print("linear speed = ", self.linear_speed)
            print("angular speed = ", self.angular_speed)
        # print(val)

    def callback(self, data):
        pose = Pose()
        if len(data.detections) > 0:
            x = data.detections[0].pose.pose.pose.position.x
            y = data.detections[0].pose.pose.pose.position.y
            z = data.detections[0].pose.pose.pose.position.z
            x_a = data.detections[0].pose.pose.pose.orientation.x
            y_a = data.detections[0].pose.pose.pose.orientation.y
            z_a = data.detections[0].pose.pose.pose.orientation.z
        # print('x: ', x, 'y: ', y, 'z: ', z)
        # print('x_a: ', x_a, 'y_a: ', y_a, 'z_a: ', z_a)
      
    

            # Transform final pose of robot to pose in base footprint at time t=0
            if self.check == True:

                point_tag_in_c = tf2_geometry_msgs.PointStamped()   # pose of tag in camera frame
                point_tag_in_c.header.frame_id = 'camera_rgb_optical_frame'
                point_tag_in_c.header.stamp = rospy.get_rostime()
                point_tag_in_c.point.x = x
                point_tag_in_c.point.y = y
                point_tag_in_c.point.z = z
        


                point_tag_in_bf = self.tfBuffer.transform(point_tag_in_c, 'base_footprint',  # pose of tag in base footprint
                                                    rospy.Duration(1.0))

                # print(point_tag_in_bf)

                # world frame is same as base_footprint at time T= 0

                point_world = tf2_geometry_msgs.PointStamped()
                point_world.header.frame_id = 'tag_0'
                point_world.header.stamp = rospy.get_rostime()
                point_world.point.x = 0.0
                point_world.point.y = -point_tag_in_bf.point.z    # since height difference remains same throughout the trajectory
                point_world.point.z = 0.12
        


                point_world = self.tfBuffer.transform(point_world, 'base_footprint',
                                                    rospy.Duration(1.0))


                # print(point_world)
                


                #initial pose is identity because robot is initially at base footprint
                initial_pose = np.array([[1, 0, 0],
                                        [0, 1, 0],
                                        [0, 0, 1]])
                
                final_pose = np.array([[1, 0, point_world.point.x],
                        [0, 1, point_world.point.y],
                        [0, 0, 1]])

                self.time = 5 # time to reach, you can change this value here to see different behaviour of turtlebot.
                
                self.inverse_kinematics(final_pose, initial_pose, self.time)
                # no need to convert z value because we are working in 2-d plane
                self.check = False

        
    def april_tag(self):

        
        rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.callback)
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        rate = rospy.Rate(10) # 10hz
        val = False
        
        

        while True:
            vel_cmd = Twist()
            vel_cmd.linear.x = 0
            vel_cmd.linear.y = 0
            vel_cmd.linear.z = 0
            vel_cmd.angular.x = 0
            vel_cmd.angular.y = 0
            vel_cmd.angular.z = 0

            start = time.time()
            if self.time != 0:
                while (time.time() - start) <self.time:
                    vel_cmd.linear.x = self.linear_speed
                    vel_cmd.angular.z = self.angular_speed
                    pub.publish(vel_cmd)
                    val = True
                    rate.sleep()


            vel_cmd.linear.x = 0.0
            vel_cmd.angular.z = 0
            pub.publish(vel_cmd)
            rate.sleep()

            if val == True:
                break
        


if __name__ == '__main__':
    try:
        rospy.init_node('april_tag', anonymous=True)

        assignment_2 = Assignment_2()
        assignment_2.april_tag()
    except rospy.ROSInterruptException:
        pass
    