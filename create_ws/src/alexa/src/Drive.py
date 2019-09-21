import rospy
import time

from geometry_msgs.msg import Twist

class Drive():
    """Drives the Create 2

    To use:
    >>> drive = Drive()
    >>> drive.forward(speed=0.5)

    """

    def __init__(self):
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


    def forward(self, speed=0.5):
        print(f"moving forward at speed {speed}")

        vel_msg = Twist()
        vel_msg.linear.x = abs(speed)
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        
        self.velocity_publisher.publish(vel_msg)

        time.sleep(1)

    def spin(self, speed=60, angle=360):
        print(f"spinning {angle} degrees at {speed}")

        vel_msg = Twist()
        PI = 3.14159
        angular_speed = speed * 2 * PI/360
        relative_angle = angle * 2 * PI/360
        
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = abs(angular_speed)

        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while(current_angle < relative_angle):
            self.velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed * (t1-t0)
        
        self.stop()

    def stop(self):
        print(f"stopping movement")

        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        self.velocity_publisher.publish(vel_msg) # forces stop

        time.sleep(1)