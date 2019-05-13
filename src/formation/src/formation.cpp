/*
 * formation.cpp
 *
 *  Created on: Apr 18, 2017
 *      Author: zeroun
 */




#include "formation.h"

#define TARGET_HEIGHT_SIMU  2
#define TARGET_ID 9   //target的ID号
int currentAngleCount;
smarteye::Formation::Formation(int argc, char** argv, const char * name)
{

    sleep(1);
    int a =1.0;
    systemID = -1;
    updateHz = 20;
    float updateTime = 1.0/updateHz;
    ros::init(argc, argv, "formation");
    nh = boost::make_shared<ros::NodeHandle>("~");
    std::string viconName;
    nh->getParam("viconName", viconName);
    /********初始化参数***************/
    uavState = YGC_HOVER;
    bool IsGotParam;

    IsGotParam = nh->getParam("system_id",systemID);
    if(IsGotParam)
    {
        ROS_INFO("got system id successfully and id is %d",systemID);

    }
    else
    {
        ROS_ERROR("failed to get system id and node is shut down");
        exit(0);
    }
    std::string uavName;
    uavName = "/uav"+num2str(systemID);
    IsGotParam = nh->getParam("SimuFlag",IsUseSimu);
    if(IsGotParam)
    {
        if(IsUseSimu) //仿真模式
        {
            ROS_INFO("This formation node is in simulation mode!");
            localPoseSub = nh->subscribe(uavName+"/mavros/local_position/pose", 10, &smarteye::Formation::ReceiveLocalPose,this);


        }
        else //实物飞行模式
        {
            ROS_INFO("This formation node  is in real fly mode");
            uwbPositionSubsciber = nh->subscribe("uwb_bridge/uwb_data", 10, &smarteye::Formation::uwbPositionReceived,this);
            currentUWBPositionPublisher = nh->advertise<geometry_msgs::PoseStamped>(uavName+"/mavros/vision_pose/pose",10);
            uwbUpdateTimer = nh->createTimer(ros::Duration(0.03),&Formation::uwbUpdate, this);
            rangeSub = nh->subscribe(uavName+"/mavros/distance_sensor/hrlv_ez4_pub", 10,
                                     &smarteye::Formation::ReceiveLocalRange, this);
        }
    }
    else
    {
        ROS_ERROR("failed to get simulation flag and node is shut down");
        exit(0);
    }

    setPositionPub = nh->advertise<geometry_msgs::PoseStamped>
            (uavName+"/mavros/setpoint_position/local",10);
    px4StateSub = nh->subscribe(uavName+"/mavros/state", 10,&smarteye::Formation::ReceiveStateInfo, this);
    formationUpdateTimer = nh->createTimer(ros::Duration(updateTime),
                                           &Formation::update, this);

    arming_client = nh->serviceClient<mavros_msgs::CommandBool>(uavName+"/mavros/cmd/arming");
    setModeClient = nh->serviceClient<mavros_msgs::SetMode>(uavName+"/mavros/set_mode");
    takoffClient = nh->serviceClient<mavros_msgs::SetMode>(uavName+"/mavros/cmd/takeoff");

    keyboardSub= nh->subscribe("/keyboard/keydown",1,&Formation::ReceiveKeybdCmd,this);


    setVelPub = nh->advertise<geometry_msgs::TwistStamped>(uavName+"/mavros/setpoint_velocity/cmd_vel", 10);
    allPoseSub = nh->subscribe("/ygc/allPosition",3,&smarteye::Formation::ReceiveAllPose,this);
    currentAngleCount =0;
    InitParam();

}

smarteye::Formation::~Formation()
{

}

std::string smarteye::Formation::num2str(int i)
{
    std::stringstream ss;
    ss<<i;
    return ss.str();
}





void smarteye::Formation::ReceiveStateInfo(const mavros_msgs::State::ConstPtr &msg)
{
    px4State = *msg;
}

void smarteye::Formation::ReceiveKeybdCmd(const keyboard::Key &key)
{

    switch(key.code)
    {
    case 'a':   //arm the vehicle
    {
        if(px4State.armed)
        {


            ROS_WARN("the vehicle %d is already armed!",systemID);
        }
        else
        {
            positionSet = localPose;
            positionSet.pose.position.z = -1;
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = true;
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle %d armed",systemID);
            }
            else
            {
                ROS_WARN("failed to arm vehicle %d",systemID);
            }
        }

        initPose.position.x = localPose.pose.position.x;
        initPose.position.y = localPose.pose.position.y;
        initPose.position.z = localPose.pose.position.z;
        ROS_INFO("initial height is %f",initPose.position.z);
        break;
    }
    case 'd':   //disarm
    {
        if(!px4State.armed)
        {

            ROS_WARN("the vehicle %d is already disarmed",systemID);
        }
        else
        {
            positionSet = localPose;
            positionSet.pose.position.z = -1;
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = false;
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle %d disarmed",systemID);
            }
            else
            {
                ROS_WARN("failed to disarm vehicle %d",systemID);
            }

        }
        break;
    }
    case 'o':   //set offboard mode
    {
        positionSet = localPose;
        positionSet.pose.position.z = -1;
        uavState = YGC_HOVER;
        mavros_msgs::SetMode setmodeCMD;
        setmodeCMD.request.custom_mode = "OFFBOARD";
        if(setModeClient.call(setmodeCMD) && setmodeCMD.response.mode_sent)
        {
            ROS_INFO("the mode of vehicle %d has changed to offboard",systemID);
        }

        else
        {
            ROS_WARN("vehicle %d failed to set offboard mode!",systemID);
        }
        break;
    }
    case 't':     //takeoff
    {
        heiCtr.ei = 0;
        ROS_INFO("vehicle %d is going to take off",systemID);
        if(localPose.pose.position.z > 0.4)
        {
            ROS_WARN("the vehicle %d is already takeoff!",systemID);
        }
        else
        {
            positionSet.pose.position.x = localPose.pose.position.x;
            positionSet.pose.position.y = localPose.pose.position.y;
            positionSet.pose.orientation.w = localPose.pose.orientation.w;
            positionSet.pose.orientation.x = localPose.pose.orientation.x;
            positionSet.pose.orientation.y = localPose.pose.orientation.y;
            positionSet.pose.orientation.z = localPose.pose.orientation.z;
            ROS_INFO("current x is %f",localPose.pose.position.x);
            ROS_INFO("current y is %f",localPose.pose.position.y);
            uavState = YGC_TAKEOFF;
        }
        break;
    }
    case 'l':    //land
    {
        heiCtr.ei = 0;
        ROS_INFO("vehicle %d is going to land",systemID);
        positionSet.pose.position.x = localPose.pose.position.x;
        positionSet.pose.position.y = localPose.pose.position.y;
        positionSet.pose.orientation.w = localPose.pose.orientation.w;
        positionSet.pose.orientation.x = localPose.pose.orientation.x;
        positionSet.pose.orientation.y = localPose.pose.orientation.y;
        positionSet.pose.orientation.z = localPose.pose.orientation.z;
        ROS_INFO("current x is %f",localPose.pose.position.x);
        ROS_INFO("current y is %f",localPose.pose.position.y);
        uavState = YGC_LAND;
        break;
    }
    case 'h':  //hover
    {
        heiCtr.ei = 0;
        ROS_INFO("vehicle %d mode is changed to hover",systemID);
        positionSet = localPose;
        uavState = YGC_HOVER;
        break;
    }
    case 'e':  //encirlcement
    {
        ROS_INFO("vehicle %d begins enciclement control",systemID);
        uavState = YGC_ENCIRCLE;
        break;
    }
    case 'c':
    {
        ROS_INFO("vehicle %d begins circle control",systemID);
        uavState = YGC_CIRCLE;
    }
    case 'w':
    {
        ROS_INFO("vehicle %d is going to position control",systemID);
        positionSet.pose.position.x = localPose.pose.position.x;
        positionSet.pose.position.y = localPose.pose.position.y;
        positionSet.pose.position.z = positionSet.pose.position.z+0.1;
        positionSet.pose.orientation.w = localPose.pose.orientation.w;
        positionSet.pose.orientation.x = localPose.pose.orientation.x;
        positionSet.pose.orientation.y = localPose.pose.orientation.y;
        positionSet.pose.orientation.z = localPose.pose.orientation.z;
        ROS_INFO("current set z is %f",localPose.pose.position.z);
        uavState = YGC_POSCTR;
        break;
    }
    case 's':
    {
        ROS_INFO("vehicle %d is going to position control",systemID);
        positionSet.pose.position.x = localPose.pose.position.x;
        positionSet.pose.position.y = localPose.pose.position.y;
        positionSet.pose.position.z = positionSet.pose.position.z-0.1;
        positionSet.pose.orientation.w = localPose.pose.orientation.w;
        positionSet.pose.orientation.x = localPose.pose.orientation.x;
        positionSet.pose.orientation.y = localPose.pose.orientation.y;
        positionSet.pose.orientation.z = localPose.pose.orientation.z;
        ROS_INFO("current set z is %f",localPose.pose.position.z);
        uavState = YGC_POSCTR;
        break;
    }
    default:
    {
        ROS_WARN("vehicle %d receives unknown command!",systemID);
    }

    }
}

void smarteye::Formation::update(const ros::TimerEvent &event)
{


    switch (uavState)
    {
    case YGC_HOVER:
    {
        setPositionPub.publish(positionSet);
        break;
    }
    case YGC_POSCTR:
    {
        setPositionPub.publish(positionSet);
        break;
    }
    case YGC_TAKEOFF:
    {

        takeoffCtr();
        break;
    }
    case YGC_LAND:
    {
        positionSet.pose.position.x = localPose.pose.position.x;
        positionSet.pose.position.y = localPose.pose.position.y;
        positionSet.pose.orientation.w = localPose.pose.orientation.w;
        positionSet.pose.orientation.x = localPose.pose.orientation.x;
        positionSet.pose.orientation.y = localPose.pose.orientation.y;
        positionSet.pose.orientation.z = localPose.pose.orientation.z;
        landCtr();
        break;
    }
    case YGC_ENCIRCLE:
    {
        if(!IsUseSimu)   //实物模式
        {
            encircleCtr(1.3+initPose.position.z);
        }
        else
        {
            encircleCtr(TARGET_HEIGHT_SIMU);
        }
        break;
    }
    case YGC_CIRCLE:
    {
        if(!IsUseSimu)   //实物模式
        {
            circleCtr(1+initPose.position.z);
        }
        else
        {
            circleCtr(TARGET_HEIGHT_SIMU+0.8*systemID);
        }
        break;
    }
    default:
    {
        ROS_WARN("Unexpected state occurs in vehicle %d",systemID);
        break;
    }
    }
}

void smarteye::Formation::uwbUpdate(const ros::TimerEvent &event)
{
    uavCurrentUWBPose.header.stamp = ros::Time::now();
    uavCurrentUWBPose.header.frame_id = "/world";
    currentUWBPositionPublisher.publish(uavCurrentUWBPose);
}

void smarteye::Formation::ReceiveLocalPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    localPose.pose.position.x = msg->pose.position.x;
    localPose.pose.position.y = msg->pose.position.y;
    localPose.pose.position.z = msg->pose.position.z;
    localPose.pose.orientation.x = msg->pose.orientation.x;
    localPose.pose.orientation.y = msg->pose.orientation.y;
    localPose.pose.orientation.z = msg->pose.orientation.z;
    localPose.pose.orientation.w = msg->pose.orientation.w;
    // Using ROS tf to get RPY angle from Quaternion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(localPose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(uavRollENU, uavPitchENU, uavYawENU);

}

void smarteye::Formation::ReceiveLocalRange(const sensor_msgs::RangeConstPtr &range)
{
    uavCurrentUWBPose.pose.position.z = range->range;
}



void smarteye::Formation::ReceiveAllPose(const bearing_common::AllPositionConstPtr &msg)
{
    allUavPos = *msg;
}








void smarteye::Formation::uwbPositionReceived(const uwb_bridge::uwbMsgConstPtr &uwb_msg)
{

    if(uwb_msg->stampID == systemID)
    {
        uavCurrentUWBPose.pose.position.x = uwb_msg->x;
        uavCurrentUWBPose.pose.position.y = uwb_msg->y;
        //currentUWBPositionPublisher.publish(uavCurrentUWBPose);

    }

}



void smarteye::Formation::takeoffCtr()
{
    if(!IsUseSimu) // 实物飞行模式
    {
        if(localPose.pose.position.z-initPose.position.z<0.8)
        {
            positionSet.pose.position.z = localPose.pose.position.z+0.15+initPose.position.z;
        }
        else
        {
            positionSet.pose.position.z = 1+initPose.position.z;
        }
    }
    else   //仿真模式
    {
        positionSet.pose.position.z = TARGET_HEIGHT_SIMU;
    }
    setPositionPub.publish(positionSet);

}

void smarteye::Formation::landCtr()
{
    if(localPose.pose.position.z>0.3)
    {

        positionSet.pose.position.z = localPose.pose.position.z-0.2;
    }
    else
    {
        positionSet.pose.position.z = -0.5;
    }
    setPositionPub.publish(positionSet);

}

void smarteye::Formation::encircleCtr(double targetHei)
{


}

void smarteye::Formation::circleCtr(double targetHei)
{
    double targetx=initPose.position.x+cos(currentAngleCount*36/(3.1415926*2));
    double targety=initPose.position.y+sin(currentAngleCount*36/(3.1415926*2));
    double error = mypower(localPose.pose.position.x-targetx)+mypower(localPose.pose.position.y-targety);
    float threshold = 0.2;
    positionSet.pose.position.x = targetx;
    positionSet.pose.position.y = targety;
    positionSet.pose.position.z = targetHei;
    setPositionPub.publish(positionSet);
    if(error<threshold)
    {
        currentAngleCount++;
        if(currentAngleCount>9)
        {
            currentAngleCount = 0;
        }
    }

}

void smarteye::Formation::InitParam()
{
    env_k_alpha = 1;
    env_k_beta = 1;
    env_k_gamma = 1.4;
    timecount = 0;
    rotTheta = -0.1;
    positionSet = localPose;
    positionSet.pose.position.z = -1;
    initPose.position.z = -0.2;
}



smarteye::heiCtrller::heiCtrller()
{
    ei = 0;
    hei_kp1 = 2;
    hei_kp2 = 0.9;
    hei_kpdiv = 0.1;
    hei_kd = 0.15;
    hei_ki = 0.01;
    hei_bias = 0.001;

}

smarteye::heiCtrller::~heiCtrller()
{

}

float smarteye::heiCtrller::cacOutput()
{

    float output;
    float err = targetHei - currentHei;
    ei = ei + hei_ki*err;
    ei = ValueLimit(ei,0.3,-0.3);
    float kpOutput;
    if (fabs(err)>hei_kpdiv)
    {
        //如果在第二段内
        if (err>0)
            kpOutput = hei_kp1*hei_kpdiv+ hei_kp2*(err-hei_kpdiv);
        else
            kpOutput = hei_kp1*hei_kpdiv + hei_kp2*(err+hei_kpdiv);
    }
    else
    {
        //如果在第一段内
        kpOutput =hei_kp1*err;
    }
    output = hei_bias + kpOutput + ei + hei_kd*(err-previousErr);
    output = ValueLimit(output,1,-1);
    previousErr = err;
    return(output);
}


float smarteye::ValueLimit(float value, float max, float min)
{
    if(value > max)
        return max;
    else if(value < min)
        return min;
    else
        return value;
}


float smarteye::CacMatrix2DNorm(Eigen::Matrix2d matrix)
{
    float norm=0.0;
    for(int i=0;i<2;i++)
    {
        for(int j=0;j<2;j++)
        {
            norm=norm+matrix(i,j)*matrix(i,j);
        }
    }
    return(sqrt(norm));
}


float smarteye::CacVector2DNorm(Eigen::Vector2d vec)
{
    float norm=0.0;
    for(int i=0;i<2;i++)
    {
        norm=norm+vec[i]*vec[i];
    }
    return(sqrt(norm));
}


smarteye::xyCtrller::xyCtrller()
{
    ei = 0;
    xy_kp = 0.2;
    xy_ki = 0.003;
    xy_kd = 0.05;
    xy_bias = 0;
}

smarteye::xyCtrller::~xyCtrller()
{

}

float smarteye::xyCtrller::cacOutput()
{

    float output;
    float err = targetPos- currentPos;
    ei = ei + xy_ki*err;
    ei = ValueLimit(ei,0.3,-0.3);
    output = xy_bias + xy_kp*err + ei + xy_kd*(err-previousErr);
    output = ValueLimit(output,1,-1);
    previousErr = err;
    return(output);

}


double smarteye::mypower(double x)
{
    return x*x;
}
