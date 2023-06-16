#!/usr/bin/env python3

import rospy
import serial
from lab4_driver.msg import imu_msg
import tf
import numpy as np
import regex as re


def euler_to_quaternion(roll, pitch, yaw):
  """
  Converts from Euler angle to a quaternion.
  roll, pitch and yaw are in radians and the output is in qx, qy, qz, qw
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

  return [qx, qy, qz, qw]


def imu_to_rosmsg(line):

    """
    changes to a list of strings 
    
    IMU msg type:
    
    std_msgs/Header Header
        uint32 seq
        time stamp
        string frame_id
    sensor_msgs/Imu IMU
    std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
    geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
    float64[9] orientation_covariance
    geometry_msgs/Vector3 angular_velocity
        float64 x
        float64 y
        float64 z
    float64[9] angular_velocity_covariance
    geometry_msgs/Vector3 linear_acceleration
        float64 x
        float64 y
        float64 z
    float64[9] linear_acceleration_covariance
    sensor_msgs/MagneticField MagField
    std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
    geometry_msgs/Vector3 magnetic_field
        float64 x
        float64 y
        float64 z
    float64[9] magnetic_field_covariance

    """


    line1 = line

    if line == '' or line1[1] == '' or line1[2] == '' or line1[3] == '' or line1[4] =='' :
        rospy.logwarn("Erroneous IMU DATA. Incomplete values being sent by the device")
        return None 
   
    # print(line1)

    # print("test")
    # print("time", utm_data[0])
    msg = imu_msg()
    now = rospy.get_rostime()

    #header data
    msg.Header.stamp.secs = now.secs
    msg.Header.stamp.nsecs = now.nsecs   #nsecs wont matter as we will be using it at 1hz
    msg.Header.frame_id = "IMU1_Frame"
    # print(line1[1])

    #orientation
    yaw = float(line1[1]) * (np.pi/180)
    pitch = float(line1[2]) * (np.pi/180)
    roll = float(line1[3]) * (np.pi/180)
    # print("roll, pitch, yaw", roll, pitch, yaw )
    # print(tf.transformations.quaternion_from_euler(roll, pitch, yaw))
    quaternion = euler_to_quaternion(roll, pitch, yaw)
    # print(quaternion)
    msg.IMU.orientation.x = quaternion[0]
    msg.IMU.orientation.y = quaternion[1]
    msg.IMU.orientation.z = quaternion[2]
    msg.IMU.orientation.w = quaternion[3]

    #magnetic field
    msg.MagField.magnetic_field.x = float(line1[4])
    msg.MagField.magnetic_field.y = float(line1[5])
    msg.MagField.magnetic_field.z = float(line1[6])

    #acceleration
    msg.IMU.linear_acceleration.x = float(line1[7])
    msg.IMU.linear_acceleration.y = float(line1[8])
    msg.IMU.linear_acceleration.z = float(line1[9])

    #angular rates
    vel_x = re.split(r'[`\=~!@#$%^&*()_\[\]{};\'\\:"|<,/<>?]', line1[10])
    vel_y = re.split(r'[`\=~!@#$%^&*()_\[\]{};\'\\:"|<,/<>?]', line1[11])
    vel_z = re.split(r'[`\=~!@#$%^&*()_\[\]{};\'\\:"|<,/<>?]', line1[12])

    msg.IMU.angular_velocity.x = float(vel_x[0])
    msg.IMU.angular_velocity.y = float(vel_y[0])
    msg.IMU.angular_velocity.z = float(vel_z[0])

    return msg


    


if __name__ == '__main__':
    rospy.init_node('imu_data')
    serial_port = rospy.get_param('~port','/dev/ttyUSB0')
    serial_baud = rospy.get_param('~baudrate',115200) 
    sampling_rate = rospy.get_param('~sampling_rate',5.0)
    
    port = serial.Serial(serial_port, serial_baud, timeout=3.)
    
    rospy.logdebug("Using IMU on port "+serial_port+" at "+str(serial_baud))
    print("Using IMU on port "+serial_port+" at "+str(serial_baud))
    # print(port)

    port.write(b'$VNWRG,07,40*XX\r')
    rospy.sleep(2)

    imu_pub = rospy.Publisher('imu', imu_msg, queue_size=10)

    try:
        while not rospy.is_shutdown():
            try:
                line = port.readline()
                # print(line)
                
                if line == '':
                    rospy.logwarn("NO IMU DATA")
                else:
                    line = line.decode("utf-8") #converts bytes to string  utf-8
                    line1 = line.split(",")
                    if ('$VNYMR' in line1[0]):
                        imu_data = imu_to_rosmsg(line1)
                        if not imu_data is None:
                            imu_pub.publish(imu_data)

            except: #this except is to handle when characters come between the numbers
                continue

    except rospy.ROSInterruptException:
        port.close()
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down imu_data node...")



