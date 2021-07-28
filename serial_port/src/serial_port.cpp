#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <serial_port/Range.h>
#include <serial_port/BDSephem.h>
#include <serial_port/Gprmc.h>
#include <iostream>
#include <cstdlib>

using std::cout;
using std::cin;
using std::endl;
using std::string;
using std::vector;
using std::strtol;
using std::strtof;

string check_msg(const string &);

bool get_data_body(const string &data, vector<string> *);

serial_port::Range get_range_msg(const vector<string> &, int);

sensor_msgs::Imu get_imu_msg(const vector<string> &, int);

serial_port::BDSephem get_ephem_msg(const vector<string> &, int);

serial_port::Gprmc get_gprmc_msg(const vector<string> &, int);

vector<string> switch_time_zone(vector<string> gprmc_body,int time_zone);

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
    GNSS(const std::string &port, int baudrate) : _port(port), _baud_rate(baudrate), _initialized(false) {
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
    void send_command(const std::string &command) {
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
    serial::Serial *_serial;
    bool _initialized;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "serial_port");
    // 创建句柄
    ros::NodeHandle n;
    // publisher
    ros::Publisher range_pub = n.advertise<serial_port::Range>("range", 1000);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 1000);
    ros::Publisher ephem_pub = n.advertise<serial_port::BDSephem>("ephem", 1000);
    ros::Publisher gprmc_pub = n.advertise<serial_port::Gprmc>("gprmc", 1000);
    // 默认参数
    string serial_port;
    int serial_baudrate;
    string imu_frame_id_param;
    float imu_frequency;    //　s/次
    float range_frequency;  //　s/次
    string ephem_frequency = "onchanged";
    float gprmc_frequency;  //　s/次
    int time_zone;

    // 从参数服务器获取值
    // 串口参数
    n.param("/serial_port/serial_port", serial_port, std::string("/dev/ttyUSB2"));
    n.param("/serial_port/serial_baudrate", serial_baudrate, 115200);
    // 接收机参数
    n.param("imu_frame_id_param", imu_frame_id_param, std::string("gnss"));
    n.param("/serial_port/time_zone", time_zone, 0);
    n.param("/serial_port/imu_frequency", imu_frequency, 0.01f);
    n.param("/serial_port/range_frequency", range_frequency, 0.05f);
    n.param("/serial_port/ephem_frequency", ephem_frequency);
    n.param("/serial_port/gprmc_frequency", gprmc_frequency, 1.f);


    // 新建一个 GNSS 实例：以后改进成单例
    GNSS gnss(serial_port, serial_baudrate);
    if (!gnss.is_initialized()) {
        return -1;
    }
    // 原始数据的发送频率
    std::string imu_command, range_command, ephem_command,gprmc_command;
    imu_command = std::string("log rawimua ontime ") + std::to_string(imu_frequency);
    range_command = std::string("log rangea ontime ") + std::to_string(range_frequency);
    ephem_command = std::string("log bdsephemerisa ") + ephem_frequency;
    gprmc_command = std::string("log gprmc ontime ") + std::to_string(gprmc_frequency);
    gnss.send_command(imu_command);
    gnss.send_command(range_command);
    gnss.send_command(ephem_command);
    gnss.send_command(gprmc_command);

    ros::Rate loop_rate(500); //HZ
    int count_i = 0, count_r = 0, count_e = 0, count_g = 0;
    ROS_INFO("Waiting for new message... ");

    while (ros::ok()) {
        std::string data = gnss.read_line();
        if (data.empty()) continue;
        //ROS_INFO("\ndata = %sdata.size() = %lu \n" ,data.c_str(), data.size());
        if (check_msg(data) == "RANGEA") {
            // 将数据存进 range_body
            vector<string> range_body;
            bool check = get_data_body(data, &range_body);
            if (!check) continue;
            /* debug /
            ROS_ERROR("debug range_body");
            for(auto c : range_body)
                cout << c << " ";
            cout << endl;
            ROS_ERROR("========debug fields=======");
            / debug */
            // 发布 range_msg
            serial_port::Range range_msg = get_range_msg(range_body, count_r);
            range_pub.publish(range_msg);
            count_r++;
        }
        else if (check_msg(data) == "RAWIMUA") {
            // 将数据存进 imu_body
            vector<string> imu_body;
            bool check = get_data_body(data, &imu_body);
            if (!check) continue;
            // 发布 range_msg
            sensor_msgs::Imu imu_msg = get_imu_msg(imu_body, count_i);
            imu_pub.publish(imu_msg);
            count_i++;
        } else if (check_msg(data) == "BDSEPHEMERISA") {
            // 将数据存进 ephem_body
            vector<string> ephem_body;
            bool check = get_data_body(data, &ephem_body);
            if (!check) continue;
            /* debug /
            ROS_ERROR("debug ephem_body");
            for(auto c : ephem_body)
                cout << c << " ";
            cout << endl;
            ROS_ERROR("========debug ephem=======");
            / debug */
            // 发布 ephem_msg
            serial_port::BDSephem ephem_msg = get_ephem_msg(ephem_body, count_e);
            ephem_pub.publish(ephem_msg);
            count_e++;
        } else if (check_msg(data) == string("GPRMC")) {
            // 将数据存进 ephem_body
            vector<string> gprmc_body;
            bool check = get_data_body(data,&gprmc_body);
            if(!check) continue;
            // 时区转换
            if (time_zone) {
                gprmc_body = switch_time_zone(gprmc_body, time_zone);
            }
            /* debug /
            ROS_ERROR("debug gprmc_body");
            for(auto c : gprmc_body)
                cout << c << " ";
            cout << endl;
            ROS_ERROR("========debug gprmc=======");
            / debug */
            // 发布 gprmc_msg
            serial_port::Gprmc gprmc_msg = get_gprmc_msg(gprmc_body, count_g);
            gprmc_pub.publish(gprmc_msg);
            count_g++;
        } else if (check_msg(data) == string("Empty data")) {
            continue;
        } else ROS_WARN("Unkonwn data, will not be processed: %s", data.c_str());

        loop_rate.sleep();
    }

    return 0;
}

// 检查获取的数据类型
string check_msg(const string &data) {
    string check = "Empty data";
    if (data.size() > 70) {
        //find 返回所查找字符的下标，当查找的字符不存在时返回 -1 ，而返回 0 时说明字符存在且下标为1
        if(data.find('#')+1){
            check = data.substr(data.find('#') + 1, data.find(',') - data.find('#') - 1);
        } else if (data.find('$') + 1) {
            check = data.substr(data.find('$') + 1, data.find(',') - data.find('$') - 1);
        }
    }
    return check;
}

// 获取有效数据
bool get_data_body(const string &data, vector<string> *fields) {
    // 获取数据 body
    int begin = data.find(';'), end = data.rfind('*');
    string content = data.substr(begin + 1, end - begin - 1)+string(",*EndID"); //添加末尾标识符
    // 以逗号为区分，将各个字段的数据存进 fields
    if (data.size() > 70) {
        int next_comma;
        while (!content.empty()) {
            next_comma = content.find(',');
            fields->emplace_back(content.substr(0, next_comma));
            content = content.substr(next_comma + 1);
            if(content == "*EndID") break;
        }
    }
    else {
        string header = check_msg(data);
        for( auto &c : header)
            c = tolower(c);
        ROS_WARN("false get %s_body",header.c_str());
        return false;
    }

    return true;
}

// 将 range 数据转换成 ros 消息
serial_port::Range get_range_msg(const vector<string> &range_body, int count) {
    //用于初始化range_msg
    serial_port::RangeInformation init;
    init.prn_number = 0;
    init.glofreq = 0;
    init.psr = 0.0;
    init.psr_std = 0.0;
    init.adr = 0.0;
    init.adr_std = 0.0;
    init.dopp = 0.0;
    init.noise_density_ratio = 0.0;
    init.locktime = 0.0;
    init.tracking_status = 0;
    int32_t numb_of_observ = strtol(range_body[0].c_str(), nullptr, 10);
//    vector<serial_port::RangeInformation> tmp(numb_of_observ, init);

    //声明range_msg
    serial_port::Range range_msg;
    //初始化range_msg
    range_msg.info = vector<serial_port::RangeInformation>(numb_of_observ, init);

    // 将range数据赋值给range_msg
    range_msg.header.frame_id = "range_frame";
    range_msg.header.seq = count;
    range_msg.header.stamp = ros::Time::now();
    range_msg.numb_of_observ = numb_of_observ;
    for (int i = 0, j = 0; j < numb_of_observ; j++) {
        range_msg.info[j].prn_number = strtol(range_body[i + 1].c_str(), nullptr, 10);
        range_msg.info[j].glofreq = strtol(range_body[i + 2].c_str(), nullptr, 10);
        range_msg.info[j].psr = strtof(range_body[i + 3].c_str(), nullptr);
        range_msg.info[j].psr_std = strtof(range_body[i + 4].c_str(), nullptr);
        range_msg.info[j].adr = strtof(range_body[i + 5].c_str(), nullptr);
        range_msg.info[j].adr_std = strtof(range_body[i + 6].c_str(), nullptr);
        range_msg.info[j].dopp = strtof(range_body[i + 7].c_str(), nullptr);
        range_msg.info[j].noise_density_ratio = strtof(range_body[i + 8].c_str(), nullptr);
        range_msg.info[j].locktime = strtof(range_body[i + 9].c_str(), nullptr);
        range_msg.info[j].tracking_status = strtol(range_body[i + 10].c_str(), nullptr, 10);
    }
    /* debug /
    ROS_WARN("tag:range_msg.info");
    for(auto i : range_msg.info)
        cout << i << " ";
    cout << endl;
    ROS_WARN("==============================split line============================");
     / debug */

    return range_msg;

}

// 将 imu 数据转换成 ros 消息
sensor_msgs::Imu get_imu_msg(const vector<string> &imu_body, int count) {
    sensor_msgs::Imu imu_msg;
    //header
    imu_msg.header.seq = count;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "imu_frame";
    //acc
    imu_msg.linear_acceleration.x = strtof(imu_body[5].c_str(), nullptr) * (0.200 / 65536) / 125;
    imu_msg.linear_acceleration.y = strtof(imu_body[4].c_str(), nullptr) * (0.200 / 65536) / 125;
    imu_msg.linear_acceleration.z = strtof(imu_body[3].c_str(), nullptr) * (0.200 / 65536) / 125;
    //gyro
    imu_msg.angular_velocity.x = strtof(imu_body[8].c_str(), nullptr) * (0.008 / 65536) / 125;
    imu_msg.angular_velocity.y = strtof(imu_body[7].c_str(), nullptr) * (0.008 / 65536) / 125;
    imu_msg.angular_velocity.z = strtof(imu_body[6].c_str(), nullptr) * (0.008 / 65536) / 125;
    //orientation
    imu_msg.orientation.x = 1;
    imu_msg.orientation.y = 0;
    imu_msg.orientation.z = 0;
    imu_msg.orientation.w = 0;
    return imu_msg;
}

// 将 ephemreis 数据转换成 ros 消息
serial_port::BDSephem get_ephem_msg(const vector<string> &ephem_body, int count) {
    serial_port::BDSephem ephem_msg;
    ephem_msg.sat = strtol(ephem_body[0].c_str(), nullptr, 10);
    ephem_msg.week = strtol(ephem_body[1].c_str(), nullptr, 10);
    ephem_msg.ura = strtof(ephem_body[2].c_str(), nullptr);
    ephem_msg.health = strtol(ephem_body[3].c_str(), nullptr, 10);
    ephem_msg.tgd1 = strtof(ephem_body[4].c_str(), nullptr);
    ephem_msg.tgd2 = strtof(ephem_body[5].c_str(), nullptr);
    ephem_msg.AODC = strtol(ephem_body[6].c_str(), nullptr, 10);
    ephem_msg.toc = strtol(ephem_body[7].c_str(), nullptr, 10);
    ephem_msg.a0 = strtof(ephem_body[8].c_str(), nullptr);
    ephem_msg.a1 = strtof(ephem_body[9].c_str(), nullptr);
    ephem_msg.a2 = strtof(ephem_body[10].c_str(), nullptr);
    ephem_msg.AODE = strtol(ephem_body[11].c_str(), nullptr, 10);
    ephem_msg.toe = strtol(ephem_body[12].c_str(), nullptr, 10);
    ephem_msg.RootA = strtof(ephem_body[13].c_str(), nullptr);
    ephem_msg.ecc = strtof(ephem_body[14].c_str(), nullptr);
    ephem_msg.omg = strtof(ephem_body[15].c_str(), nullptr);
    ephem_msg.delta_N = strtof(ephem_body[16].c_str(), nullptr);
    ephem_msg.M0 = strtof(ephem_body[17].c_str(), nullptr);
    ephem_msg.OMG_0 = strtof(ephem_body[18].c_str(), nullptr);
    ephem_msg.OMG = strtof(ephem_body[19].c_str(), nullptr);
    ephem_msg.i0 = strtof(ephem_body[20].c_str(), nullptr);
    ephem_msg.I_DOT = strtof(ephem_body[21].c_str(), nullptr);
    ephem_msg.cuc = strtof(ephem_body[22].c_str(), nullptr);
    ephem_msg.cus = strtof(ephem_body[23].c_str(), nullptr);
    ephem_msg.crc = strtof(ephem_body[24].c_str(), nullptr);
    ephem_msg.crs = strtof(ephem_body[25].c_str(), nullptr);
    ephem_msg.cic = strtof(ephem_body[26].c_str(), nullptr);
    ephem_msg.cis = strtof(ephem_body[27].c_str(), nullptr);

    return ephem_msg;
}

// 将 gprmc 数据转换成 ros 消息
serial_port::Gprmc get_gprmc_msg(const vector<string> &grpmc_body, int count) {
    serial_port::Gprmc gprmc_msg;
    gprmc_msg.utc = grpmc_body[1];
    gprmc_msg.pos_status = grpmc_body[2];
    gprmc_msg.latitude = strtof(grpmc_body[3].c_str(),nullptr);
    gprmc_msg.lat_dir = grpmc_body[4];
    gprmc_msg.longitude = strtof(grpmc_body[5].c_str(),nullptr);
    gprmc_msg.lon_dir = grpmc_body[6];
    gprmc_msg.speed = strtof(grpmc_body[7].c_str(),nullptr);
    gprmc_msg.track = strtof(grpmc_body[8].c_str(),nullptr);
    gprmc_msg.date = grpmc_body[9];
    gprmc_msg.mag_var = strtof(grpmc_body[10].c_str(),nullptr);
    gprmc_msg.var_dir = grpmc_body[11];
    gprmc_msg.mode_ind = grpmc_body[12];

    return gprmc_msg;
}

vector<string> switch_time_zone(vector<string> gprmc_body,int time_zone){
    // 获取当前时间
    float time = strtof(gprmc_body[1].c_str(), nullptr);
    // 世界标准时加上时区时差
    time += float(time_zone * 10000);

    /// 时间溢出处理
    // 每个月的天数
    vector<uint> day_num = {0,31,28,31,30,31,30,31,31,30,31,30,31};
    // 获取当前年月日
    int year = strtol(gprmc_body[9].substr(4,2).c_str(), nullptr, 10);
    int month = strtol(gprmc_body[9].substr(2, 2).c_str(), nullptr, 10);
    int day = strtol(gprmc_body[9].substr(0, 2).c_str(), nullptr, 10);
    // 闰年的 2 月有 29天
    if (year % 4 == 0){
        day_num[2] = 29;
    }

    // hour 溢出处理
    if (time > 240000.0){
        time -= float(240000.0);
        day += 1;
    } else if(time < 0) {
        time += float(240000.0);
        day -= 1;
    }
    // day 溢出处理
    if (day > day_num[month]){
        day = 1;
        month += 1;
    } else if(day <= 0) {
        day = day_num[month-1];
        month -= 1;
    }
    // month 溢出处理
    if (month > 12) {
        month = 1;
        year += 1;
    } else if (month <= 0){
        month = 12;
        year -= 1;
    }
    // year 溢出处理
    if (year >= 100) {
        year = 1;
    } else if (year < 0) {
        year = 99;
    }
    string time_str = std::to_string(time);
    size_t time_lenth = time_str.substr(0,time_str.find('.')).size();
    if(time_lenth < 6) {
        time_str = string("0") + time_str;
    }
    gprmc_body[1] = time_str;

    string day_str = std::to_string(day);
    string month_str = std::to_string(month);
    string year_str = std::to_string(year);
    if (day_str.size() < 2) {
        day_str = string("0") + day_str;
    }
    if (month_str.size() < 2) {
        month_str = string("0") + month_str;
    }
    if (year_str.size() < 2) {
        year_str = string("0") + year_str;
    }
    gprmc_body[9] = day_str + month_str + year_str;

    return gprmc_body;
}


