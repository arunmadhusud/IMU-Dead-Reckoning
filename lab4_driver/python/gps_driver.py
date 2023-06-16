#!/usr/bin/env python3

import rospy
import serial
import utm
from lab4_driver.msg import gps_msg
"""
Code to test the conversion from lat-long to UTM with a real GPS

"""


def basic_unit_conv_utm(latitude, longitude, time_stamp):

    """
    latitude and longitude are in the format DDMM.MMMMM
    To convert from NMEA to decimal format:
    DD = int(float(Lat)/100) 
    MM = float(lat) - DD * 100
    Example:
    Lat : 3137.36664 = 31 degrees and 37.36664 seconds
    = 31 + 37.36664/60 = 31.6227773

    Time stamp is in hhmmss.ss.
    example: 202134.00
    We need to convert it into secs
    
    """
    lat_dd = int(latitude/100)
    lat_mm = latitude - lat_dd*100
    lat = lat_dd + (lat_mm/60)

    longit_dd = int(longitude/100)
    longit_mm = longitude - longit_dd*100
    longit = longit_dd + (longit_mm/60)


    time_stamp_hh = int(int(time_stamp)/10000)
    time_stamp_mm = (int((int(time_stamp)/10000 - time_stamp_hh)*100))
    time_stamp_ss = (((int(time_stamp)/10000 - time_stamp_hh)*100)%1)*100
    time_stamp_secs = 3600 * time_stamp_hh + 60*time_stamp_mm + time_stamp_ss
    # print(time_stamp, "time_stamp")
    # print("time stamp _Seconds " , time_stamp_secs)

    time_stamp_nsecs = ((time_stamp) % 1) * (10 ** 9)

    return (lat,-longit), (time_stamp_secs, time_stamp_nsecs)


def convert_to_utm(line):

    """
    changes to a list of strings 
    ['$GPGGA', '004212.747', '4217.6336', 'N', '07106.8660', 'W', '1', '04', '2.2', '-10.2', 'M', '-33.8', 'M', '', '0000*78\r\n']
     latitude = 4217.6336/100 ; longitude = 07106.8660/100
    """

    
    line1 = line

    if line == '' or line1[1] == '' or line1[2] == '' or line1[3] == '' or line1[4] =='' :
        rospy.logwarn("Erroneous GPS DATA. Incomplete values being sent by the device")
        return None 
   

    time_stamp = float(line1[1])
    lat = float(line1[2])
    longit = float(line1[4]) 

    conv_data = basic_unit_conv_utm(lat, longit, time_stamp)
    lat = conv_data[0][0]
    longit = conv_data[0][1]
    time_stamp = conv_data[1]
    altitude = float(line1[9])

    print("latitude:", lat, " longitude:", longit)
    (easting, northing, zone_number, zone_letter) = utm.from_latlon(lat, longit)
    print("easting, northing, zone_number, zone_letter", easting, northing, zone_number, zone_letter)
    return (time_stamp, easting, northing, zone_number, zone_letter, lat, longit, altitude)


def utm_to_rosmsg(utm_data):

    """
    gps_msgs type:
    std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
    float64 altitude
    float64 latitude
    float64 longitude
    float64 easting
    float64 northing
    float64 zone_number
    string zone_letter
    """

    print("test")
    # print("time", utm_data[0])
    msg = gps_msg()
    msg.Header.stamp.secs = int(utm_data[0][0])
    msg.Header.stamp.nsecs = int(utm_data[0][1])   #nsecs wont matter as we will be using it at 1hz
    msg.Header.frame_id = "GPS1_Frame"
    msg.Altitude = utm_data[7]
    msg.Latitude = utm_data[5]
    msg.Longitude = utm_data[6]
    msg.UTM_easting = utm_data[1]
    msg.UTM_northing = utm_data[2]
    msg.Zone = utm_data[3]
    msg.Letter = utm_data[4]

    return msg


    


if __name__ == '__main__':
    rospy.init_node('gps_data')
    serial_port = rospy.get_param('~port','/dev/pts/3')
    serial_baud = rospy.get_param('~baudrate',4800)
    sampling_rate = rospy.get_param('~sampling_rate',5.0)
    
    port = serial.Serial(serial_port, serial_baud, timeout=3.)
    rospy.logdebug("Using GPS on port "+serial_port+" at "+str(serial_baud))

    sleep_time = 0.2

    gps_pub = rospy.Publisher('gps', gps_msg, queue_size=10)

    try:
        while not rospy.is_shutdown():
            msg = gps_msg()
            # print(msg)
            
            line = port.readline()
            
            # print(line)
            if line == '':
                rospy.logwarn("NO GPS DATA")
            else:
                line = line.decode("utf-8") #converts bytes to string
                line1 = line.split(",")
                # print("starts with", line.startswith('$GPGGA'))
                # print(line)
                # print(line1)
                # print("0th part of list" ,'$GPGGA' in line1[0])
                if ('$GPGGA' in line1[0]):
                # if line.startswith('$GPGGA'):
                    print("test2")
                    utm_coords = convert_to_utm(line1)
                    if not utm_coords is None:
                        ros_data_gps = utm_to_rosmsg(utm_coords)
                        print(ros_data_gps)
                        gps_pub.publish(ros_data_gps)

            rospy.sleep(sleep_time)   

    except rospy.ROSInterruptException:
        port.close()
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down gps_data node...")



