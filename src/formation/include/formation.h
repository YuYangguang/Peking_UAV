/*
 * formation.h
 *
 *  Created on: Apr 18, 2017
 *      Author: zeroun
 */

#ifndef FORMATION_H_
#define FORMATION_H_

#include <ros/ros.h>




#include "../bearing_common/include/3rd_party_header.h"
#include "uwb_bridge/uwbMsg.h"
#include "sensor_msgs/Range.h"
#define YGC_HOVER 0
#define YGC_TAKEOFF 1
#define YGC_LAND 2
#define YGC_ENCIRCLE 3
#define YGC_CIRCLE 4
#define YGC_POSCTR 5
namespace smarteye {
float ValueLimit(float value,float max,float min);
class heiCtrller
{
public:
    heiCtrller();
    ~heiCtrller();
    double currentHei;
    double targetHei;

    float hei_kp1;
    float hei_kp2;
    float hei_kpdiv;
    float hei_ki;
    float hei_kd;
    float hei_bias;
    float previousErr;
    float ei;  //控制器积分项
    float cacOutput();
};

class xyCtrller
{
public:
    xyCtrller();
    ~xyCtrller();
    double currentPos;
    double targetPos;
    float xy_kp;
    float xy_ki;
    float xy_kd;
    float xy_bias;
    float previousErr;
    float ei;  //控制器积分项
    float cacOutput();
};
float CacMatrix2DNorm(Eigen::Matrix2d matrix);
float CacVector2DNorm(Eigen::Vector2d vec);

double  mypower(double x);  //乘方函数,返回x^2


class Formation
{
public:
    Formation(int argc,char** argv,const char * name);
    ~Formation();
    double uavRollENU, uavPitchENU, uavYawENU;
    ros::Timer  formationUpdateTimer;
    ros::Timer  uwbUpdateTimer;
    boost::shared_ptr<ros::NodeHandle> nh;
    std::string num2str(int i);
    bool IsUseSimu;


    bearing_common::AllPosition allUavPos;
    heiCtrller heiCtr;
    xyCtrller xCtr;
    xyCtrller yCtr;
    ros::ServiceServer receiveHParamSrv;
    bool receiveHParamSetSrv(bearing_common::HParamSetSrv::Request &req, bearing_common::HParamSetSrv::Response &res);
    void ReceiveStateInfo(const mavros_msgs::State::ConstPtr& msg);
    void ReceiveKeybdCmd(const keyboard::Key &key);

private:
    int updateHz;
    int timecount;
    int systemID;
    float rotTheta;  //速度方向补偿角度
    float env_k_alpha;
    float env_k_beta;
    float env_k_gamma;
    int uavState;


    geometry_msgs::Pose initPose;
    geometry_msgs::TwistStamped selfVel;  //自身速度
    geometry_msgs::TwistStamped neighborVel;  //邻居智能体速度
    ros::ServiceClient paramClient;
    ros::ServiceClient arming_client;
    ros::ServiceClient setModeClient;
    ros::ServiceClient takoffClient;
    mavros_msgs::State px4State;
    void update(const ros::TimerEvent& event);
    void uwbUpdate(const ros::TimerEvent& event);
    ros::Publisher setPositionPub;
    ros::Publisher tarVelCoorPub;  //目标速度估计协调量发布
    ros::Publisher currentUWBPositionPublisher;
    ros::Publisher setVelPub;
    ros::Subscriber agentVelSub;

    ros::Subscriber keyboardSub;
    ros::Subscriber px4StateSub;
    ros::Subscriber localPoseSub;
    ros::Subscriber rangeSub;

    ros::Subscriber allPoseSub;
    ros::Subscriber selfVelSub;
    ros::Subscriber neighborVelSub;
    ros::Subscriber uwbPositionSubsciber;
    geometry_msgs::PoseStamped desiredPose;
    geometry_msgs::PoseStamped positionSet;
    geometry_msgs::TwistStamped velocitySet;
    geometry_msgs::PoseStamped localPose;
    geometry_msgs::PoseStamped uavCurrentUWBPose;
    nav_msgs::Odometry targetInfo;
    sensor_msgs::Range localRangeData;

    void ReceiveLocalPose(const geometry_msgs::PoseStampedConstPtr& msg);
    void ReceiveLocalRange(const sensor_msgs::RangeConstPtr& range);
    void ReceiveAllPose(const bearing_common::AllPositionConstPtr &msg);
    void ReceiveAgentVel(const bearing_common::GroupBearingConstPtr &msg);
    void uwbPositionReceived(const uwb_bridge::uwbMsgConstPtr& vicon_msg);
    void takeoffCtr();
    void landCtr();
    void encircleCtr(double targetHei);  //输入，期望的高度
    void circleCtr(double targetHei);    //输入，期望高度
    void InitParam(void);


};

}



#endif /* FORMATION_H_ */
