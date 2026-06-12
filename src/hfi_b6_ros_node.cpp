#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <dynamic_reconfigure/server.h>
#include <handsfree_ros_imu/HandsfreeRosImuConfig.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

static bool detail_diagnostic_enable = false;
static sensor_msgs::Imu imu_msg;
static double angle_degree[3] = {};
static bool serial_connected = false;

static bool check_sum(const uint8_t* data, size_t len, uint8_t check)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < len; ++i)
        sum += data[i];
    return sum == check;
}

static void hex_to_short(const uint8_t* raw, int16_t* out)
{
    for (int i = 0; i < 4; ++i)
        std::memcpy(&out[i], &raw[i * 2], sizeof(int16_t));
}

static int open_serial(const std::string& port, int baudrate)
{
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
        return -1;

    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0)
    {
        close(fd);
        return -1;
    }

    speed_t baud;
    switch (baudrate)
    {
        case 9600:    baud = B9600;    break;
        case 19200:   baud = B19200;   break;
        case 38400:   baud = B38400;   break;
        case 57600:   baud = B57600;   break;
        case 115200:  baud = B115200;  break;
        case 230400:  baud = B230400;  break;
        case 460800:  baud = B460800;  break;
        case 500000:  baud = B500000;  break;
        case 576000:  baud = B576000;  break;
        case 921600:  baud = B921600;  break;
        case 1000000: baud = B1000000; break;
        default:
            ROS_ERROR("Unsupported baudrate: %d", baudrate);
            close(fd);
            return -1;
    }

    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        close(fd);
        return -1;
    }

    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    tcflush(fd, TCIOFLUSH);
    return fd;
}

static void diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    if (detail_diagnostic_enable)
    {
        stat.addf("Linear Acc X", "%f", imu_msg.linear_acceleration.x);
        stat.addf("Linear Acc Y", "%f", imu_msg.linear_acceleration.y);
        stat.addf("Linear Acc Z", "%f", imu_msg.linear_acceleration.z);
        stat.addf("Orientation Roll", "%f", angle_degree[0]);
        stat.addf("Orientation Pitch", "%f", angle_degree[1]);
        stat.addf("Orientation Yaw", "%f", angle_degree[2]);
        stat.addf("Orientation X", "%f", imu_msg.orientation.x);
        stat.addf("Orientation Y", "%f", imu_msg.orientation.y);
        stat.addf("Orientation Z", "%f", imu_msg.orientation.z);
        stat.addf("Orientation W", "%f", imu_msg.orientation.w);
    }
    else
    {
        stat.addf("Orientation Yaw", "%f", angle_degree[2]);
    }

    if (serial_connected)
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
    else
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "IMU disconnected");
}

static void reconfigure_callback(handsfree_ros_imu::HandsfreeRosImuConfig& config, uint32_t)
{
    ROS_INFO("Configuration received.");
    detail_diagnostic_enable = config.detail_diagnostic_enable_;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    dynamic_reconfigure::Server<handsfree_ros_imu::HandsfreeRosImuConfig> dyncfg_server;
    dyncfg_server.setCallback(boost::bind(&reconfigure_callback, _1, _2));

    diagnostic_updater::Updater updater;
    updater.setHardwareID("AGV05");
    updater.add("Status", diagnostic_callback);

    std::string port;
    int baudrate;
    int rate;
    std::string frame_id;
    pnh.param<std::string>("port", port, "/dev/ttyUSB0");
    pnh.param<int>("baudrate", baudrate, 921600);
    pnh.param<int>("rate", rate, 250);
    pnh.param<std::string>("frame_id", frame_id, "base_link");

    imu_msg.header.frame_id = frame_id;

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("handsfree/imu", 10);

    ros::Rate loop_rate(rate);

    int fd = -1;
    uint8_t buff[11] = {};
    int key = 0;
    double angular_velocity[3] = {};
    double acceleration[3] = {};
    bool pub_flag[3] = {true, true, true};

    while (ros::ok())
    {
        updater.update();

        if (fd < 0)
        {
            fd = open_serial(port, baudrate);
            if (fd >= 0)
            {
                serial_connected = true;
                ROS_INFO("%s opened", port.c_str());
            }
            else
            {
                serial_connected = false;
                ROS_ERROR_ONCE("Open %s failed: %s", port.c_str(), strerror(errno));
                ros::Duration(1.0).sleep();
                continue;
            }
        }

        uint8_t read_buf[512];
        int n = read(fd, read_buf, sizeof(read_buf));
        if (n < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                loop_rate.sleep();
                ros::spinOnce();
                continue;
            }
            ROS_ERROR("%s disconnected: %s", port.c_str(), strerror(errno));
            close(fd);
            fd = -1;
            serial_connected = false;
            ros::Duration(1.0).sleep();
            continue;
        }
        else if (n == 0)
        {
            loop_rate.sleep();
            ros::spinOnce();
            continue;
        }

        for (int i = 0; i < n; ++i)
        {
            buff[key] = read_buf[i];
            key++;

            if (buff[0] != 0x55)
            {
                key = 0;
                continue;
            }
            if (key < 11)
                continue;

            int16_t shorts[4];

            if (buff[1] == 0x51 && pub_flag[0])
            {
                if (check_sum(buff, 10, buff[10]))
                {
                    hex_to_short(&buff[2], shorts);
                    for (int j = 0; j < 3; ++j)
                        acceleration[j] = shorts[j] / 32768.0 * 16 * 9.8;
                }
                else
                {
                    ROS_WARN("Checksum error (0x%02x)", buff[1]);
                }
                pub_flag[0] = false;
            }
            else if (buff[1] == 0x52 && pub_flag[1])
            {
                if (check_sum(buff, 10, buff[10]))
                {
                    hex_to_short(&buff[2], shorts);
                    for (int j = 0; j < 3; ++j)
                        angular_velocity[j] = shorts[j] / 32768.0 * 2000 * M_PI / 180.0;
                }
                else
                {
                    ROS_WARN("Checksum error (0x%02x)", buff[1]);
                }
                pub_flag[1] = false;
            }
            else if (buff[1] == 0x53 && pub_flag[2])
            {
                if (check_sum(buff, 10, buff[10]))
                {
                    hex_to_short(&buff[2], shorts);
                    for (int j = 0; j < 3; ++j)
                        angle_degree[j] = shorts[j] / 32768.0 * 180.0;
                }
                else
                {
                    ROS_WARN("Checksum error (0x%02x)", buff[1]);
                }
                pub_flag[2] = false;
            }
            else
            {
                ROS_WARN("Unknown (0x%02x)", buff[1]);
                key = 0;
                continue;
            }

            key = 0;
            if (pub_flag[0] || pub_flag[1] || pub_flag[2])
                continue;
            pub_flag[0] = pub_flag[1] = pub_flag[2] = true;

            ros::Time stamp = ros::Time::now();
            imu_msg.header.stamp = stamp;

            double angle_radian[3];
            for (int j = 0; j < 3; ++j)
                angle_radian[j] = angle_degree[j] * M_PI / 180.0;

            tf2::Quaternion qua;
            qua.setRPY(angle_radian[0], angle_radian[1], angle_radian[2]);

            imu_msg.orientation.x = qua.x();
            imu_msg.orientation.y = qua.y();
            imu_msg.orientation.z = qua.z();
            imu_msg.orientation.w = qua.w();

            imu_msg.angular_velocity.x = angular_velocity[0];
            imu_msg.angular_velocity.y = angular_velocity[1];
            imu_msg.angular_velocity.z = angular_velocity[2];

            imu_msg.linear_acceleration.x = acceleration[0];
            imu_msg.linear_acceleration.y = acceleration[1];
            imu_msg.linear_acceleration.z = acceleration[2];

            imu_pub.publish(imu_msg);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    if (fd >= 0)
        close(fd);

    return 0;
}
