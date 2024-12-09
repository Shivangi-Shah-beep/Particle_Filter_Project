import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class ImuandOdom():
    def __init__(self):
        rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=10)
        self.imu_data=None
        self.odom_data=None
    
    def imu_callback(self,msg):
        self.imu_data=msg
        if self.imu_data is not None:
            rospy.loginfo(f"The imu data is : {self.imu_data}")
        else:
            rospy.logwarn("No imu data recieved")
        
    def odom_callback(self,msg):
        self.odom_data=msg
        if self.odom_data is not None:
            rospy.loginfo(f"The odom data is : {self.odom_data}")
        else:
            rospy.logwarn("No imu data recieved")

def main():
    rospy.init_node('extract_imu_data', anonymous=True)
    node=ImuandOdom()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
