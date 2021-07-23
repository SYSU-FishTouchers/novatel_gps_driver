#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>

#include "novatel_gps_driver/novatel_sentence.h"
#include "novatel_gps_driver/parsers/corrimudata.h"
#include <novatel_gps_driver/parsers/header.h>
 
int main(int argc, char** argv)
{
        ros::init(argc, argv, "serial_port");
        //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
        ros::NodeHandle n;
        ros::Publisher imu_pub = n.advertise<novatel_gps_msgs::NovatelCorrectedImuData>("imu", 1000);
        //创建一个serial类
        serial::Serial sp;
        //创建timeout
        serial::Timeout to = serial::Timeout::simpleTimeout(100);
        //设置要打开的串口名称
        sp.setPort("/dev/ttyUSB1");
        //设置串口通信的波特率
        sp.setBaudrate(115200);
        //串口设置timeout
        sp.setTimeout(to);
    
        try
        {
            //打开串口
            sp.open();
        }
        catch(serial::IOException& e)
        {
            ROS_ERROR_STREAM("Unable to open port.");
            return -1;
        }
        
        //判断串口是否打开成功
        if(sp.isOpen())
        {
            ROS_INFO_STREAM("/dev/ttyUSB1 is opened.");
        }
        else
        {
            return -1;
        }
        
        ros::Rate loop_rate(500);


        while(ros::ok())
        {
            //获取缓冲区内的字节数
            size_t n = sp.available();
          
            unsigned char buffer[1024];                    
       
            //读出数据
            n=sp.read(buffer, n);
           // sp.read(buffer);

           
            if(n!=0)
            {
                    std::cout << "缓存区字节数目：" << n <<std::endl;
                    // //打印缓存数据
                    // for(int i=0; i<n; i++)
                    // {
                    //     std::cout << buffer[i] ;                       
                    // }
                    // std::cout << std::endl;
                              
                    int fenhao;
                    int xinghao;                    
                    
                    for(int i=0;i<n;i++)
                    {                         
                        if(buffer[i]==';')
                        {
                            fenhao = i;
                        }             
                        if(buffer[i]=='*')
                        {
                            xinghao = i;
                            break;
                        }       
                    }

                    //提取出body数据
                    int sub;
                    char body[150];
                    sub = xinghao - fenhao;
                    for(int i=0;i<sub;i++)
                    {
                        body[i] = buffer[fenhao+1+i];
                    }
                   
                    for(int i=0; i<sub; i++)
                    {
                        std::cout << body[i] ;                       
                    }
                     std::cout << std::endl;

                     //区分字段
                     int douhao[9];
                     int j=0;
                     for(int i=0;i<n;i++)
                    {
                        if(body[i]==',')                      
                        {                            
                            douhao[j]=i;
                            j++;
                        }   
                        if(body[i]=='*')
                        {
                            douhao[8] = i;
                            break;
                        }
                    }

                    //拆分字段
                     std::string gps_week_num,gps_seconds,pitch_rate,roll_rate,yaw_rate,lateral_acceleration,longitudinal_acceleration,vertical_acceleration;
                    
                     for(int i=0;i<=douhao[0]-1;i++)
                     {
                         gps_week_num += body[i];
                     }
                    std::cout<< "gps_week_num:" << gps_week_num << std::endl;
                    
                    for(int i=0;i<douhao[1]-douhao[0]-1;i++)
                     {
                         gps_seconds += body[douhao[0]+i+1];
                     }
                    std::cout<< "gps_seconds:" << gps_seconds << std::endl;            
                
                    for(int i=0;i<douhao[3]-douhao[2]-1;i++)
                     {
                         vertical_acceleration += body[douhao[2]+i+1];
                     }
                    std::cout<< "vertical_acceleration:" << vertical_acceleration << std::endl;

                    for(int i=0;i<douhao[4]-douhao[3]-1;i++)
                     {
                         longitudinal_acceleration += body[douhao[3]+i+1];
                     }
                    std::cout<< "longitudinal_acceleration:" << longitudinal_acceleration << std::endl;

                    for(int i=0;i<douhao[5]-douhao[4]-1;i++)
                     {
                         lateral_acceleration += body[douhao[4]+i+1];
                     }
                    std::cout<< "lateral_acceleration:" << lateral_acceleration << std::endl;

                    for(int i=0;i<douhao[6]-douhao[5]-1;i++)
                     {
                         yaw_rate += body[douhao[5]+i+1];
                     }
                    std::cout<< "yaw_rate:" << yaw_rate << std::endl;

                    for(int i=0;i<douhao[7]-douhao[6]-1;i++)
                     {
                         pitch_rate += body[douhao[6]+i+1];
                     }
                    std::cout<< "pitch_rate:" << pitch_rate << std::endl;

                    for(int i=0;i<douhao[8]-douhao[7]-1;i++)
                     {
                         roll_rate += body[douhao[7]+i+1];
                     }
                    std::cout<< "roll_rate:" << roll_rate << std::endl;
                    // std::cout << std::endl;

                    novatel_gps_driver::NovatelSentence  sentence;

                    if(sentence.body.empty())
                    {
                        std::cout << "里面没有东西" << std::endl;
                    }

                    sentence.body.push_back(gps_week_num);
                    sentence.body.push_back(gps_seconds);
                    sentence.body.push_back(pitch_rate);
                    sentence.body.push_back(roll_rate);
                    sentence.body.push_back(yaw_rate);
                    sentence.body.push_back(lateral_acceleration);
                    sentence.body.push_back(longitudinal_acceleration);
                    sentence.body.push_back(vertical_acceleration);
                   
                   if(!sentence.body.empty())
                    {
                       for(auto i :sentence.body)
                            std::cout << i <<" ";
                        std::cout << std::endl;
                    }

                    // const novatel_gps_driver::NovatelSentence &abc = sentence;
                    // novatel_gps_driver::CorrImuDataParser parse;

                    // novatel_gps_msgs::NovatelCorrectedImuDataPtr imu_msg = parse.ParseAscii(abc);

                    std::cout << "abc = " << abc << std::endl;
                  
                    // imu_pub.publish(imu_msg);



                    //把数据发送回去
                    // sp.write(buffer, n);
                    
                }
                    loop_rate.sleep();
        }
        
        //关闭串口
        sp.close();
    
        return 0;
}


