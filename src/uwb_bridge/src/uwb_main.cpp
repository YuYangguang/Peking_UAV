#include "ros/ros.h"
#include "std_msgs/String.h"
#include "uwb_parse.h"
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/types.h>
#include <assert.h>
#include <string.h> //strcpy
#include <map>
#include <vector>
#include <fstream>
#include <unistd.h>
int DATA_COUNT=0;



int main(int argc, char **argv)
{

    ROS_INFO("start uwb_node process");
    ros::Time::init();
    ros::init(argc, argv, "uwb_bridge");
    boost::shared_ptr<ros::NodeHandle> nh = boost::make_shared<ros::NodeHandle>();
    ros::Publisher uwbPub = nh->advertise<uwb_bridge::uwbMsg>("/uwb_bridge/uwb_data",5);
    int socket_desc,rcv_size;
    int err=-1;
    socklen_t optlen;
    struct sockaddr_in server;//定义服务器的相关参数
    char server_reply[5000];

    //Create socket
    //下面的AF_INET也可以用PF_INET。AF_INET主要是用于互联网地址，而 PF_INET 是协议相关，通常是sockets和端口
    socket_desc = socket(AF_INET , SOCK_STREAM , 0);//第二个参数是套接口的类型：SOCK_STREAM或SOCK_DGRAM。第三个参数设置为0。
    if (socket_desc == -1)
    {
        printf("Could not create socket");
    }
    rcv_size = 4*640000;    /* 接收缓冲区大小为4*640K */
    optlen = sizeof(rcv_size);
    err = setsockopt(socket_desc,SOL_SOCKET,SO_RCVBUF, (char *)&rcv_size, optlen);//设置套接字，返回值为-1时则设置失败
    if(err<0){
        printf("设置接收缓冲区大小错误\n");
    }
    server.sin_addr.s_addr = inet_addr("192.168.0.201");//服务器IP地址
    server.sin_family = AF_INET;//对应与socket，也可选PF_INET
    server.sin_port = htons( 6666 );//端口号
    if (connect(socket_desc , (struct sockaddr *)&server , sizeof(server)) < 0)//用建立的socket尝试同设置好的服务器connect
    {
        perror("connect error:");
        return 1;
    }
    ROS_INFO("Connected");

    char recbuff[8000];//接收的数据缓存大小，这是我自己设置的区域，为了存放报文
    unsigned long buffsize;
    int packetCount = 0;

    buffsize=0;//该值标示当前缓存数据的大小，及下一帧数据存放的地址
    while (ros::ok())
    {
        int readSize = recv(socket_desc, server_reply , sizeof(server_reply), 0);//从服务器接收数据，其中第三个参数为单次接收的数据大小

        if(readSize == 73)
        {
            DATA_COUNT++;
            if(DATA_COUNT>300)
            {
                DATA_COUNT = 0;
                ROS_INFO("The data is received well.");
            }
            //ROS_INFO("read size is %d",readSize);
        }
        else
        {
            //ROS_INFO("read size is %d",readSize);
            continue;
        }
        //同上面的rcv_size区分开，上面的rcv_size是TCP/IP的机理，传输过程中，数据会暂时先存储在rcv_size里.
        //然后你recv再从rcv_size这个缓冲里取你设置的sizeof（serve_reply）的数据。其中readSize为recv的返回值，该值返回你实际接收到的数据大小，这点要注意。
        //接收到的数据放在server_reply[5000]里面
        if(readSize < 0)
        {//实际接收到的数据为负，表示接收出错
            printf("recv failed");
            //should shut down connection and reconnect!
            assert(false);
            return 1;
        }
        else if (readSize == 0)
        {//表示无数据传输，接收到数据为0
            printf("readSize=0");
            break;
        }
        ++packetCount;//接收到包的次数加1

        memcpy(recbuff + buffsize, server_reply, readSize);//memcpy为内存拷贝函数，将该次接收到的server_reply的数据拷贝到recbuff里
        //其中+buffsize，从recbuff头地址+buffsize的地址开始拷贝（第一次buffsize=0，及从头开始拷贝）拷贝的大小为readSize，即recv实际接收到的数据大小。
        buffsize += readSize;//buffsize=buffsize+readSize，此时buffsize指向该次拷贝的数据大小的下一位。
        const int packetHeadSize = 8;//定义控制域的数据大小为8个字节
        static int expectedPacketSize = -1;//定义期望得到的数据包大小为-1，以此判断本次接收到的数据是否是数据头部
        if (expectedPacketSize == -1 && buffsize >= packetHeadSize)
        {//接收到的数据比8个字节大，即包含了控制域及数据域，则进行下一步分析
            char tempbuff[4];
            memcpy(tempbuff, recbuff, 4);
            int32_t len = char2int32(tempbuff,sizeof(tempbuff),true);

            //assert(len == 69);//判断是否为00000045H，是否符合启动字符条件，注意小端对齐
            if (!(len ==69))//如果不是00000045H，则表示该次数据有误
            {
                ROS_ERROR("the length is not correct,is %d\n",len);
                continue;
            }


            //start token must be PRES, otherwise reconnect!
            char header[4];
            memcpy(header, recbuff + 4, 4);//收取第5到第8个字节报文,判断是否是"PRES"
            char header_stadard[4]={'P','R','E','S'};
            int IsMatched = strcmp(header,header_stadard);    //若header和期望的一致，则函数strcmp返回0,否则返回1或者-1
            //assert(IsMatched);//判断是否为PRES，是否符合启动字符条件，注意小端对齐
            if (IsMatched！=0)//如果不是PRES，则表示该次数据有误
            {
                ROS_ERROR("The header of the data is not correct! The received header is %s\n",header);
                continue;
            }

            expectedPacketSize = 73;//则期望得到的数据包大小为报文长度加控制域长度
        }

        //get one whole packet!
        if (expectedPacketSize != -1 && buffsize >= expectedPacketSize)
        {//当期望得到的数据包大小不是-1
            // 并且recbuff里接收到的数据大小已经大于所需要的数据大小，如果接收到的数据小于完整报文的长度，则继续接收
            //
            //下面为接收到的完整的一帧报文，定义了一个解包函数负责解包，从缓存数据的第9个字节开始取，取完整数据域长度的数据，即只取数据域的内容
            char rawData[64];
            memcpy(rawData, recbuff + 8, 64);//提取数据段
            uwbData_t uwbdata;
            data_parse(rawData, &uwbdata);
            uwb_bridge::uwbMsg rosMsg = trans2ros(uwbdata);
            rosMsg.header.stamp=ros::Time::now();
            rosMsg.x = -rosMsg.x;
            rosMsg.y = -rosMsg.y;
            uwbPub.publish(rosMsg);
            //下面的memmove函数是内存移动函数，将下一帧报文移动到recbuff的起始处，覆盖掉已经取出的数据
            memmove(recbuff, recbuff + expectedPacketSize, buffsize - expectedPacketSize);
            buffsize -= expectedPacketSize;
            expectedPacketSize = -1;
        }
    }



    ros::spin();
    return 0;

}
