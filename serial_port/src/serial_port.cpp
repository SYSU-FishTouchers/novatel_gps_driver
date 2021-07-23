#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <serial/serial.h>
#include <iostream>
#include <cstdlib>

using std::cout;
using std::cin;
using std::endl;
using std::string;

/*
 * 写在最前：
 *
 * 1. 严格要求按照本编译器的提示改进代码风格，最后代码不包含任何 Error 和 Warning ，具体表现在右上角只有一个绿勾
 * 2. 编写注释时，不限制中英文，但是要求紧接着中文的英文单词或符号、数字，需要左右空开一个空格。比如 a 和 1 或者 like this 。
 * 3. 编写注释时，双斜线、三斜线等表明当行为注释的符号后面，要跟随一个空格
 * 4. 如非特殊需要，不允许连续空行，只允许最多空一行。一般要求不同的函数、不同用途的变量声明语句需要用空行隔开，具体见下面 GNSS 类
 * 5. 一般不允许单行代码过长，参考本编辑界面的右侧竖线
 * 6. 有没提及的疑惑的地方，及时联系孔阳讨论
 *
 */

class GNSS {
public:
    GNSS(const std::string& port, int baudrate) : _port(port), _baud_rate(baudrate), _initialized(false) {
        _serial = new serial::Serial;
        // 设置要打开的串口端口名称
        _serial->setPort(port);
        // 设置串口通信的波特率
        _serial->setBaudrate(baudrate);

        // 串口设置 timeout
        serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
        _serial->setTimeout(timeout);

        // 打开串口
        try {
            _serial->open();
        }
        catch (serial::IOException &e) {
            ROS_ERROR("Unable to connect device via port: %s", _port.c_str());
        }

        // 判断串口是否打开成功
        if (_serial->isOpen()) {
            ROS_INFO("Successful to connect device via %s, with baudrate = %d", _port.c_str(), _baud_rate);

            // 记录初始化成功
            _initialized = true;

            // 取消所有 log 命令
            send_command("unlogall true");
        } else {
            ROS_ERROR("Unable to connect device via port: %s", _port.c_str());
        }
    }

    ~GNSS() {
        _serial->close();
        delete _serial;
        _serial = nullptr;
    }

    // 只有当可以通过此串口发送数据时才认为是初始化完成
    // 完成初始化时，会取消接收所有 log 数据
    bool is_initialized() const {
        return _initialized;
    }

    // 发送命令
    void send_command(const std::string& command) {
        // 如果没完成初始化，不允许调用此函数
        if (!_initialized) return;

        auto n = _serial->write(command + "\r\n");
        ROS_INFO("[Send %lu bytes] %s", n, command.c_str());
    }

    // 查看缓存区是否有数据
    bool is_available() const {
        // 不关心具体有多少数据在缓存区，只要可读就是 available
        return (_serial->available() > 0);
    }

    // 接收数据
    std::string read_line() {
        // 读取一行，最长不超过 65546 字节，结束符为 <CR><LF> = \r\n

        return _serial->readline(65536, "\r\n");
    }

private:
    std::string _port;
    int _baud_rate;

    // 仅声明指针，构造函数中初始化
    serial::Serial* _serial;

    bool _initialized;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "serial_port");
    // 创建句柄
    ros::NodeHandle n;

    // publisher
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu_raw", 1000);

    // 默认参数
    std::string imu_frame_id_param = "gnss";
    std::string serial_port;
    int serial_baudrate;
    float imu_frequency;

    // roslaunch 参数
    // 串口参数
    n.param("/serial_port/serial_port", serial_port, std::string("/dev/ttyUSB1"));
    n.param("/serial_port/serial_baudrate", serial_baudrate, 115200);

    // 接收机参数
    n.param("/serial_port/imu_frequency", imu_frequency, 1.f);
    n.param("imu_frame_id_param", imu_frame_id_param, std::string("gnss"));

    // 新建一个 GNSS 实例：以后改进成单例
    GNSS gnss(serial_port, serial_baudrate);
    if (!gnss.is_initialized()) {
        return -1;
    }
    // 获取 imu 的裸数据（原始数据），发送频率 = imu_frequency
    std::string imu_command = std::string("log rawimua ontime ") + std::to_string(imu_frequency);
    gnss.send_command(imu_command);

    ros::Rate loop_rate(500);
    int count = 0;

    ROS_INFO("Waiting for new message");

    while (ros::ok()) {
        // 这样写会阻塞：如果收不到数据，就会一直卡在这里，直到收到新的一行数据
        std::string data = gnss.read_line();
        //if (data.empty()) continue;
        if (data.empty()){
//            ROS_WARN("data = empty");
//            gnss.send_command(imu_command);
            continue;
        }
        ROS_INFO("data = %s", data.c_str());

        //提取出线加速度和角速度等信息
        int beg = 0, end = 0;
        for (int i=0;i<data.size();i++){
            if (data[i]==';')
                beg = i;
            if(data[i]=='*')
                end = i;
        }
        std::string new_data = data.substr(beg+1,end-beg-1);
        //ROS_INFO("new_data = %s", new_data.c_str());

        //区分字段
        int douhao[8];//","
        int j = 0;
        for (int i=0; i < new_data.size(); i++) {
            if (new_data[i] == ',') {
                douhao[j] = i+1;
                j++;
            }
        }
        double x_acc = 0.0, y_acc = 0.0, z_acc = 0.0;
        double x_gyro = 0.0, y_gyro = 0.0, z_gyro = 0.0;

        if(new_data.size()>65){
            z_acc = atof(new_data.substr(douhao[2],douhao[3]-douhao[2]).c_str());
            z_acc = z_acc * (0.200 / 65536) / 125;
            y_acc = atof(new_data.substr(douhao[3],douhao[4]-douhao[3]).c_str());
            y_acc = y_acc * (0.200 / 65536) / 125;
            x_acc = atof(new_data.substr(douhao[4],douhao[5]-douhao[4]).c_str());
            x_acc = x_acc * (0.200 / 65536) / 125;
            z_gyro = atof(new_data.substr(douhao[5],douhao[6]-douhao[4]).c_str());
            z_gyro = z_gyro * (0.008 / 65536) / 125;
            y_gyro = atof(new_data.substr(douhao[6],douhao[7]-douhao[4]).c_str());
            y_gyro = y_gyro * (0.008 / 65536) / 125;
            x_gyro = atof(new_data.substr(douhao[7]).c_str());
            x_gyro = x_gyro * (0.008 / 65536) / 125;
        }
        else{
            ROS_WARN("new_data.size=%lu",new_data.size());
        }

        /*debug
        ROS_INFO("z_acc: %f", z_acc);
        ROS_INFO("y_acc: %f", y_acc);
        ROS_INFO("x_acc: %f", x_acc);
        ROS_INFO("z_gyro: %f", z_gyro);
        ROS_INFO("y_gyro: %f", y_gyro);
        ROS_INFO("x_gyro: %f\n\n", x_gyro);
        */

        sensor_msgs::Imu imu_msg;
        //header
        imu_msg.header.seq = count;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = imu_frame_id_param;
        //acc
        imu_msg.linear_acceleration.x = x_acc;
        imu_msg.linear_acceleration.y = y_acc;
        imu_msg.linear_acceleration.z = z_acc;
        //gyro
        imu_msg.angular_velocity.x = x_gyro;
        imu_msg.angular_velocity.y = y_gyro;
        imu_msg.angular_velocity.z = z_gyro;
        //orientation
        imu_msg.orientation.x = 1;
        imu_msg.orientation.y = 0;
        imu_msg.orientation.z = 0;
        imu_msg.orientation.w = 0;

        imu_pub.publish(imu_msg);
        count++;

        loop_rate.sleep();
    }

    return 0;
}


