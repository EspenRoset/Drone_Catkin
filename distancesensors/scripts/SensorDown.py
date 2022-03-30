import rospy
from std_msgs.msg import String
import qwiic
import time
import sys

ToF = qwiic.QwiicVL53L1X(41, BusId=1)
ToF.sensor_init()
ToF.set_distance_mode(2)



def GetDistance():
    pub = rospy.Publisher('distance_down', String, queue_size=10)
    rospy.init_node('VL53L1X', anonymous=True)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        try:
            ToF.start_ranging()  # Write configuration bytes to initiate measurement
            time.sleep(.005)
            distance = ToF.get_distance()  # Get the result of the measurement from the sensor
            time.sleep(.005)
            ToF.stop_ranging()

            rospy.loginfo(str(distance))
            pub.publish(str(distance))
            rate.sleep()

        except Exception as e:
            print(e)
if __name__ == '__main__':
    try:
        GetDistance()
    except rospy.ROSInterruptException:
        pass
