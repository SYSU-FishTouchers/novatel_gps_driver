#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <serial_port/Range.h>
#include <serial_port/BDSephem.h>
#include <iostream>
#include <cstdlib>

using std::cout;
using std::cin;
using std::endl;
using std::string;
using std::vector;

string check_msg(const string &);

bool get_data_body(const string &data,vector<string> * );

serial_port::Range get_range_msg(const vector<string> &,int );

vector<float> get_imu_body(const string &);

sensor_msgs::Imu get_imu_msg(const vector<float> &,int );

serial_port::BDSephem get_ephem_msg(const vector<string> &,int);
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
    ros::Publisher range_pub = n.advertise<serial_port::Range>("range", 1000);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu",1000);
    ros::Publisher ephem_pub = n.advertise<serial_port::BDSephem>("ephem",1000);
    // 默认参数
    std::string range_frame_id_param;
    std::string serial_port;
    int serial_baudrate;
    float range_frequency;//　s/次
    float imu_frequency;//　s/次


    // 从参数服务器获取值
    // 串口参数
    n.param("/serial_port/serial_port", serial_port, std::string("/dev/ttyUSB2"));
    n.param("/serial_port/serial_baudrate", serial_baudrate, 115200);
    // 接收机参数
    n.param("/serial_port/range_frequency", range_frequency, 1.f);
    n.param("range_frame_id_param", range_frame_id_param, std::string("gnss"));
    n.param("/serial_port/imu_frequency",imu_frequency,1.f);

    // 新建一个 GNSS 实例：以后改进成单例
    GNSS gnss(serial_port, serial_baudrate);
    if (!gnss.is_initialized()) {
        return -1;
    }
    //原始数据的发送频率
    std::string imu_command,range_command,ephem_command;
    imu_command = std::string("log rawimua ontime ") + std::to_string(imu_frequency);
    range_command = string("log rangea ontime ") + std::to_string(range_frequency);
    ephem_command = "log bdsephemerisa ontime 1";
    gnss.send_command(imu_command);
    gnss.send_command(range_command);
    gnss.send_command(ephem_command);

    ros::Rate loop_rate(500);
    int count_i = 0, count_r = 0, count_e = 0;
    ROS_INFO("Waiting for new message");

    while (ros::ok()) {
        // 这样写会阻塞：如果收不到数据，就会一直卡在这里，直到收到新的一行数据
        std::string data = gnss.read_line();
        if(!data.empty())
            ROS_INFO("data = %s",data.c_str());
        // 发布 range_msg
        if(check_msg(data)=="RANGE"){
            // 将数据存进 range_body
            vector<string> range_body;
            bool check = get_data_body(data,&range_body);
            if(!check){
                continue;
            }
            /* debug
            ROS_WARN("tag:range_body");
            //cout << range_body[douhao.size()] << endl;
            for(auto i : range_body)
                cout << i << " ";
            cout << endl;
            ROS_WARN("==============================split line============================");
            */
            serial_port::Range range_msg = get_range_msg(range_body,count_r);
            range_pub.publish(range_msg);
            count_r++;
        }// 发布 imu_msg
        else if(check_msg(data)=="RAWIMUA"){
            if(get_imu_body(data).empty()){
                continue;
            }
            vector<float> imu_body = get_imu_body(data);
            sensor_msgs::Imu imu_msg = get_imu_msg(imu_body, count_i);
            imu_pub.publish(imu_msg);
            count_i++;
        }
        else if(check_msg(data)=="BDSEPHEM"){
            // 将数据存进 range_body
            vector<string> ephem_body;
            bool check = get_data_body(data,&ephem_body);
            if(!check){
                continue;
            }
            /* debug
            ROS_WARN("tag:ephem_body");
            //cout << range_body[douhao.size()] << endl;
            for(auto aaa : ephem_body)
                cout << aaa << " ";
            cout << endl;
            ROS_WARN("==============================split line============================");
            */
            serial_port::BDSephem ephem_msg = get_ephem_msg(ephem_body,count_e);
            ephem_pub.publish(ephem_msg);
            count_e++;
        }

        loop_rate.sleep();
    }

    return 0;
}
// 检查获取的数据类型
string check_msg(const string &data){
    if(data.size()>100){
        string check = data.substr(1,7);
        if(check =="RANGEA,"){
            return "RANGE";
        }
        else if (check == "RAWIMUA"){
            return "RAWIMUA";
        }
        else if(check=="BDSEPHE"){
            return "BDSEPHEM";
        }
        else{
            //ROS_ERROR("UnKonwn data");
        }
    }
    return "CheckOut";
}

// 获取 range 和 ephem 的有效数据
bool get_data_body(const string &data,vector<string> *data_body ){
    std::string new_data;
    if(data.size()>200){
        int beg = 0, end = 0;
        for (int i=0;i<data.size();i++){
            if (data[i]==';')
                beg = i;
            if(data[i]=='*')
                end = i;
        }
        new_data = data.substr(beg+1,end-beg-1);
        //ROS_INFO("new_data = %s", new_data.c_str());

        vector<unsigned> douhao;
        int j = 0;
        for(int i=0;i<new_data.size();i++){
            if(new_data[i]==','){
                douhao.push_back(i+1);
                j++;
            }
        }
        /*debug
        ROS_INFO("douhao.size=%lu",douhao.size());
        for(auto i : douhao)
            cout << i << " ";
        cout << endl;
        ROS_WARN("==============================split line============================");
        */
        //将原始range数据存进range_body
        data_body->push_back(new_data.substr(0,douhao[0]-1));
        for(int i=0;i<int(douhao.size())-1;i++){
            data_body->push_back(new_data.substr(douhao[i],douhao[i+1]-douhao[i]-1));
        }
        data_body->push_back(new_data.substr(douhao[douhao.size()-1]));

    } else{
        return false;
    }
    return true;
}

// 将 range 数据转换成 ros 消息
serial_port::Range get_range_msg(const vector<string> &range_body,int count){
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
    int32_t numb_of_observ = atoi(range_body[0].c_str());
    vector<serial_port::RangeInformation> tmp(numb_of_observ,init);

    //声明range_msg
    serial_port::Range range_msg;
    //初始化range_msg
    range_msg.info = tmp;

    // 将range数据赋值给range_msg
    range_msg.header.frame_id = "range_frame";
    range_msg.header.seq = count;
    range_msg.header.stamp = ros::Time::now();
    range_msg.numb_of_observ = numb_of_observ;
    for(int i=0,j=0; j<numb_of_observ; j++) {
        range_msg.info[j].prn_number = atoi(range_body[i+1].c_str());
        range_msg.info[j].glofreq = atoi(range_body[i+2].c_str());
        range_msg.info[j].psr = atof(range_body[i+3].c_str());
        range_msg.info[j].psr_std = atof(range_body[i+4].c_str());
        range_msg.info[j].adr = atof(range_body[i+5].c_str());
        range_msg.info[j].adr_std = atof(range_body[i+6].c_str());
        range_msg.info[j].dopp = atof(range_body[i+7].c_str());
        range_msg.info[j].noise_density_ratio = atof(range_body[i+8].c_str());
        range_msg.info[j].locktime = atof(range_body[i+9].c_str());
        range_msg.info[j].tracking_status = atoi(range_body[i+10].c_str());
    }
    /* debug /
    ROS_WARN("tag:range_msg.info");
    for(auto i : range_msg.info)
        cout << i << " ";
    cout << endl;
    ROS_WARN("==============================split line============================");
     /  */

    return range_msg;

}

// 获取 imu 的有效数据
vector<float> get_imu_body(const string &data){
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
    float x_acc = 0.0, y_acc = 0.0, z_acc = 0.0;
    float x_gyro = 0.0, y_gyro = 0.0, z_gyro = 0.0;

    if(new_data.size()>65){
        z_acc = atof(new_data.substr(douhao[2],douhao[3]-douhao[2]).c_str());
        z_acc = float(z_acc * (0.200 / 65536) / 125);
        y_acc = atof(new_data.substr(douhao[3],douhao[4]-douhao[3]).c_str());
        y_acc = float(y_acc * (0.200 / 65536) / 125);
        x_acc = atof(new_data.substr(douhao[4],douhao[5]-douhao[4]).c_str());
        x_acc = float(x_acc * (0.200 / 65536) / 125);
        z_gyro = atof(new_data.substr(douhao[5],douhao[6]-douhao[4]).c_str());
        z_gyro = float(z_gyro * (0.008 / 65536) / 125);
        y_gyro = atof(new_data.substr(douhao[6],douhao[7]-douhao[4]).c_str());
        y_gyro = float(y_gyro * (0.008 / 65536) / 125);
        x_gyro = atof(new_data.substr(douhao[7]).c_str());
        x_gyro = float(x_gyro * (0.008 / 65536) / 125);
    }
    else{
        ROS_ERROR("new_data.size=%lu < 65",new_data.size());
        vector<float> error;
        return error;
    }
    vector<float> imu_body(6,0.0);
    imu_body[0] = x_acc;
    imu_body[1] = y_acc;
    imu_body[2] = z_acc;
    imu_body[3] = x_gyro;
    imu_body[4] = y_gyro;
    imu_body[5] = z_gyro;
    return imu_body;
}

// 将 imu 数据转换成 ros 消息
sensor_msgs::Imu get_imu_msg(const vector<float> &imu_body,int count){
    sensor_msgs::Imu imu_msg;
    //header
    imu_msg.header.seq = count;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "imu_frame";
    //acc
    imu_msg.linear_acceleration.x = imu_body[0];
    imu_msg.linear_acceleration.y = imu_body[1];
    imu_msg.linear_acceleration.z = imu_body[2];
    //gyro
    imu_msg.angular_velocity.x = imu_body[3];
    imu_msg.angular_velocity.y = imu_body[4];
    imu_msg.angular_velocity.z = imu_body[5];
    //orientation
    imu_msg.orientation.x = 1;
    imu_msg.orientation.y = 0;
    imu_msg.orientation.z = 0;
    imu_msg.orientation.w = 0;
    return imu_msg;
}

// 将 ephemreis 数据转换成 ros 消息
serial_port::BDSephem get_ephem_msg(const vector<string> &ephem_body,int count){
    serial_port::BDSephem ephem_msg;
    ephem_msg.sat = atoi(ephem_body[0].c_str());
    ephem_msg.week = atoi(ephem_body[1].c_str());
    ephem_msg.ura = atof(ephem_body[2].c_str());
    ephem_msg.health = atoi(ephem_body[3].c_str());
    ephem_msg.tgd1 = atof(ephem_body[4].c_str());
    ephem_msg.tgd2 = atof(ephem_body[5].c_str());
    ephem_msg.AODC = atoi(ephem_body[6].c_str());
    ephem_msg.toc = atoi(ephem_body[7].c_str());
    ephem_msg.a0 = atof(ephem_body[8].c_str());
    ephem_msg.a1 = atof(ephem_body[9].c_str());
    ephem_msg.a2 = atof(ephem_body[10].c_str());
    ephem_msg.AODE = atoi(ephem_body[11].c_str());
    ephem_msg.toe = atoi(ephem_body[12].c_str());
    ephem_msg.RootA = atof(ephem_body[13].c_str());
    ephem_msg.ecc = atof(ephem_body[14].c_str());
    ephem_msg.omg = atof(ephem_body[15].c_str());
    ephem_msg.delta_N = atof(ephem_body[16].c_str());
    ephem_msg.M0 = atof(ephem_body[17].c_str());
    ephem_msg.OMG_0 = atof(ephem_body[18].c_str());
    ephem_msg.OMG = atof(ephem_body[19].c_str());
    ephem_msg.i0 = atof(ephem_body[20].c_str());
    ephem_msg.I_DOT = atof(ephem_body[21].c_str());
    ephem_msg.cuc = atof(ephem_body[22].c_str());
    ephem_msg.cus = atof(ephem_body[23].c_str());
    ephem_msg.crc = atof(ephem_body[24].c_str());
    ephem_msg.crs = atof(ephem_body[25].c_str());
    ephem_msg.cic = atof(ephem_body[26].c_str());
    ephem_msg.cis = atof(ephem_body[27].c_str());

    return ephem_msg;
}


