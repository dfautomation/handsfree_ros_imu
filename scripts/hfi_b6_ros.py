#!/usr/bin/env python
# -*- coding:utf-8 -*-
import serial
import struct
import rospy
import diagnostic_updater
import dynamic_reconfigure.server
import handsfree_ros_imu.cfg.HandsfreeRosImuConfig
import math
import platform
import serial.tools.list_ports
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler

detail_diagnostic_enable = False

# 查找 ttyUSB* 设备
def find_ttyUSB():
    print('imu 默认串口为 /dev/ttyUSB0, 若识别多个串口设备, 请在 launch 文件中修改 imu 对应的串口')
    posts = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
    print('当前电脑所连接的 {} 串口设备共 {} 个: {}'.format('USB', len(posts), posts))


# 校验
def checkSum(list_data, check_data):
    return sum(list_data) & 0xff == check_data


# 16 进制转 ieee 浮点数
def hex_to_short(raw_data):
    return list(struct.unpack("hhhh", bytearray(raw_data)))


# 处理串口数据
def handleSerialData(raw_data):
    global buff, key, angle_degree, magnetometer, acceleration, angularVelocity, pub_flag
    if python_version == '2':
        buff[key] = ord(raw_data)
    if python_version == '3':
        buff[key] = raw_data

    key += 1
    if buff[0] != 0x55:
        key = 0
        return
    if key < 11:  # 根据数据长度位的判断, 来获取对应长度数据
        return
    else:
        data_buff = list(buff.values())  # 获取字典所有 value

        if buff[1] == 0x51 and pub_flag[0]:
            if checkSum(data_buff[0:10], data_buff[10]):
                acceleration = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]
            else:
                # print('0x51 校验失败')
                rospy.logwarn('Checksum error (%s)' % hex(buff[1]))
            pub_flag[0] = False

        elif buff[1] == 0x52 and pub_flag[1]:
            if checkSum(data_buff[0:10], data_buff[10]):
                angularVelocity = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180 for i in range(0, 3)]

            else:
                # print('0x52 校验失败')
                rospy.logwarn('Checksum error (%s)' % hex(buff[1]))
            pub_flag[1] = False

        elif buff[1] == 0x53 and pub_flag[2]:
            if checkSum(data_buff[0:10], data_buff[10]):
                angle_degree = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 for i in range(0, 3)]
            else:
                # print('0x53 校验失败')
                rospy.logwarn('Checksum error (%s)' % hex(buff[1]))
            pub_flag[2] = False

        else:
            # print("该数据处理类没有提供该 " + str(buff[1]) + " 的解析")
            # print("或数据错误")
            rospy.logwarn('Unknown (%s)' % hex(buff[1]))
            buff = {}
            key = 0

        buff = {}
        key = 0
        if pub_flag[0] == True or pub_flag[1] == True or pub_flag[2] == True:
            return
        pub_flag[0] = pub_flag[1] = pub_flag[2] = True
        stamp = rospy.get_rostime()

        imu_msg.header.stamp = stamp
        # imu_msg.header.frame_id = "base_link"


        angle_radian = [angle_degree[i] * math.pi / 180 for i in range(3)]
        qua = quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])

        imu_msg.orientation.x = qua[0]
        imu_msg.orientation.y = qua[1]
        imu_msg.orientation.z = qua[2]
        imu_msg.orientation.w = qua[3]

        imu_msg.angular_velocity.x = angularVelocity[0]
        imu_msg.angular_velocity.y = angularVelocity[1]
        imu_msg.angular_velocity.z = angularVelocity[2]

        imu_msg.linear_acceleration.x = acceleration[0]
        imu_msg.linear_acceleration.y = acceleration[1]
        imu_msg.linear_acceleration.z = acceleration[2]

        imu_pub.publish(imu_msg)


def handle_diagnostic_status(stat):
    global detail_diagnostic_enable

    if detail_diagnostic_enable:
        stat.add('Linear Acc X', imu_msg.linear_acceleration.x)
        stat.add('Linear Acc Y', imu_msg.linear_acceleration.y)
        stat.add('Linear Acc Z', imu_msg.linear_acceleration.z)
        stat.add('Orientation Roll', angle_degree[0])
        stat.add('Orientation Pitch', angle_degree[1])
        stat.add('Orientation Yaw', angle_degree[2])
        stat.add('Orientation X', imu_msg.orientation.x)
        stat.add('Orientation Y', imu_msg.orientation.y)
        stat.add('Orientation Z', imu_msg.orientation.z)
        stat.add('Orientation W', imu_msg.orientation.w)
    else:
        stat.add('Orientation Yaw', angle_degree[2])

    if hf_imu:
        stat.summary(diagnostic_updater.DiagnosticStatus.OK, 'OK')
    else:
        stat.summary(diagnostic_updater.DiagnosticStatus.ERROR, 'IMU disconnected')

    return stat

def handle_config(config, level):
    global detail_diagnostic_enable
    rospy.loginfo('Configuration received.')
    detail_diagnostic_enable = config.detail_diagnostic_enable_
    return config

key = 0
flag = 0
buff = {}
angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]
pub_flag = [True, True, True]


if __name__ == "__main__":
    python_version = platform.python_version()[0]

    # find_ttyUSB()
    rospy.init_node("imu")

    # setup dynamic reconfigure server
    dyncfg_server = dynamic_reconfigure.server.Server(handsfree_ros_imu.cfg.HandsfreeRosImuConfig, handle_config)

    # setup diagnostic updater
    updater = diagnostic_updater.Updater()
    updater.setHardwareID("AGV05")
    updater.add("Status", handle_diagnostic_status)

    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baudrate = rospy.get_param("~baudrate", 921600)
    rate = rospy.get_param("~rate", 250) # 250hz
    frame_id = rospy.get_param("~frame_id", "base_link")
    imu_msg = Imu()
    imu_msg.header.frame_id = frame_id
    imu_pub = rospy.Publisher("handsfree/imu", Imu, queue_size=10)

    r = rospy.Rate(rate)
    hf_imu = None
    while not rospy.is_shutdown():
        updater.update()
        if not hf_imu:
            try:
                hf_imu = serial.Serial(port=port, baudrate=baudrate, timeout=0.5)
                if hf_imu.isOpen():
                    # rospy.loginfo("\033[32m串口打开成功...\033[0m")
                    rospy.loginfo('%s opened' % port)
                else:
                    hf_imu.open()
                    # rospy.loginfo("\033[32m打开串口成功...\033[0m")
                    rospy.loginfo('Open %s success' % port)
            except Exception as e:
                # print(e)
                # rospy.loginfo("\033[31m串口打开失败\033[0m")
                rospy.logerr_once('Open %s failed: %s' % (port, e))
                if hf_imu:
                    hf_imu.close()
                    hf_imu = None
                rospy.sleep(1.)
                # exit(0)
        else:
            try:
                buff_count = hf_imu.inWaiting()
                if buff_count > 0:
                    buff_data = hf_imu.read(buff_count)
                    for i in range(0, buff_count):
                        handleSerialData(buff_data[i])
            except Exception as e:
                # print("exception:" + str(e))
                # print("imu 失去连接，接触不良，或断线")
                rospy.logerr('%s disconnected: %s' % (port, e))
                if hf_imu:
                    hf_imu.close()
                    hf_imu = None
                rospy.sleep(1.)
                # exit(0)
            else:
                r.sleep()
