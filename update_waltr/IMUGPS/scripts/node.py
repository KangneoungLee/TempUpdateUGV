#!/usr/bin/env python
# license removed for brevity

import rospy
import serial
import struct
import time
import numpy as np
from std_msgs.msg import String, Bool, Float64, Int8
from sensor_msgs.msg import NavSatFix, Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler

DegreeTran = 180/3.1415926

teensySerial = serial.Serial('/dev/TeensyCOM', baudrate=115200, timeout=0.1)

def talker():
    namespace = rospy.get_namespace()
    if namespace == "/":
        namespace = ""
    GPS_topic_name = namespace + "/Teensy/GPS"
    IMU_topic_name = namespace + "/Teensy/IMU"
    Compass_topic_name = namespace + "/Teensy/Compass"
    Compass_status_topic_name = namespace + "/Teensy/Compass_status"
    
    rospy.init_node('TeensySensors', anonymous=True)
    pubGPS = rospy.Publisher(GPS_topic_name, NavSatFix, queue_size=10)   #msg type, topic name, queue size
    pubIMU = rospy.Publisher(IMU_topic_name, Imu, queue_size=10)
    pubCompass = rospy.Publisher(Compass_topic_name, Float64, queue_size=10)
    pubCompass_status = rospy.Publisher(Compass_status_topic_name, Int8, queue_size=10)
    rate = rospy.Rate(200)  #200Hz

    while not rospy.is_shutdown():
        teensySerial.write(b'\x01')
        if teensySerial.in_waiting:
            teensy_cmd, = struct.unpack('B', teensySerial.read(1))
            if teensy_cmd == 1:
                compassCal, =  struct.unpack('B', teensySerial.read(1))
                gyroCal, = struct.unpack('B', teensySerial.read(1))
                accelCal, = struct.unpack('B', teensySerial.read(1))
                imuMsg = Imu()
                imuMsg.header.frame_id = "imu_frame"
                imuMsg.header.stamp = rospy.Time.now()
                imuMsg.orientation.x, = struct.unpack('f', teensySerial.read(4))
                imuMsg.orientation.y, = struct.unpack('f', teensySerial.read(4))
                imuMsg.orientation.z, = struct.unpack('f', teensySerial.read(4))
                imuMsg.orientation.w, = struct.unpack('f', teensySerial.read(4))
                imuMsg.angular_velocity.x, = struct.unpack('f', teensySerial.read(4))
                imuMsg.angular_velocity.y, = struct.unpack('f', teensySerial.read(4))
                imuMsg.angular_velocity.z, = struct.unpack('f', teensySerial.read(4))
                imuMsg.linear_acceleration.x, = struct.unpack('f', teensySerial.read(4))
                imuMsg.linear_acceleration.y, = struct.unpack('f', teensySerial.read(4))
                imuMsg.linear_acceleration.z, = struct.unpack('f', teensySerial.read(4))
                Compass, = struct.unpack('f', teensySerial.read(4))
                Compass = Compass + 0 #90  
                #ENU (East (x), North (y), Up (z)) if offest is 0, east is zero,  north is -90 degree  
                #if offset if 90, east is 90 degree, north is 0 degree

                if Compass > 360:
                    Compass = Compass - 360
		
                if imuMsg.orientation.x + imuMsg.orientation.y + imuMsg.orientation.z + imuMsg.orientation.w == 0:
                    rospy.loginfo('BNO055 is dead.')
                else:
                    pubIMU.publish(imuMsg)
                    pubCompass.publish(Compass)
                    pubCompass_status.publish(compassCal)
                    if compassCal == 0:
                        rospy.loginfo('Compass is not fixed.')
                    if gyroCal == 0:
                        rospy.loginfo('Gyro is not fixed.')
                    if accelCal == 0:
                        rospy.loginfo('Accel is not fixed.')

            elif teensy_cmd == 2:
                GPSfix, = struct.unpack('B', teensySerial.read(1))
                latitudeData, = struct.unpack('d', teensySerial.read(8))
                longitudeData, = struct.unpack('d', teensySerial.read(8))
                GPSmsg = NavSatFix()
                GPSmsg.header.frame_id = "gps_frame"
                GPSmsg.header.stamp = rospy.Time.now()
                GPSmsg.latitude = latitudeData
                GPSmsg.longitude = longitudeData
                if GPSfix == 0:
                    rospy.loginfo('GPS is not fixed.')
                    GPSmsg.status.status = -1
                else:
                    GPSmsg.status.status = 0
                pubGPS.publish(GPSmsg)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInternalException:
        pass
