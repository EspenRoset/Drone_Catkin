import rospy
from std_msgs.msg import String
import qwiic
import time

ToF = qwiic.QwiicVL53L1X(41, BusId=0)
ToF.sensor_init()
ToF.set_distance_mode(2)

def GetDistance():
    pub = rospy.Publisher('distance_up', String, queue_size=10)
    rospy.init_node('VL53L1X', anonymous=True)
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        try:
            ToF.start_ranging()  # Write configuration bytes to initiate measurement
            time.sleep(.005)
            distance = ToF.get_distance()  # Get the result of the measurement from the sensor
            time.sleep(.005)
            ToF.stop_ranging()

            #rospy.loginfo(str(distance))
            if ToF.get_range_status() == 0:
                pub.publish(str(distance))

            else:
                pub.publish("0")
            rate.sleep()

        except Exception as e:
            print(e)
if __name__ == '__main__':
    try:
        GetDistance()
    except rospy.ROSInterruptException:
        pass

