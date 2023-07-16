#ifndef FIRE_NAV_H_
#define FIRE_NAV_H_
#include <ros/ros.h>
#include <string>
#include <nav_msgs/OccupancyGrid.h>
#include <eigen3/Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseResult.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <std_srvs/Trigger.h>
#include <mutex>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <zmq.hpp>
#include <tf/transform_broadcaster.h>

class FireManager
{
public:
    FireManager(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    void initialize();
    void setupTimer();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    std::string WorldFrame_;
    std::string BodyFrame_;
    std::string CameraFrame_;
    std::string TargetFrame_;

    tf::TransformBroadcaster brCTT; // br camera to target .target in camera.

    uint8_t MoveBaseStatus_;
    uint8_t NavManagerStatus_; // 1 for go straight;2 for planner

    std::shared_ptr<zmq::context_t> ContextPtr_;
    std::shared_ptr<zmq::socket_t> ReqSockPtr_;

    // ReqSock.setsockopt(ZMQ_IDENTITY, "client1", 7);
    // ReqSock.connect("tcp://localhost:7001");

    int IdSeqPub_;
    ros::Publisher PlannerTargetPub_;

    ros::Subscriber NavStatusSubscriber_;
    void NavStatusCallBack(const actionlib_msgs::GoalStatusArray &msg);

    ros::Timer ReceiveAndUpdateFireStateTimer;
    void ReceiveAndUpdateFireStateTimerCallBack(const ros::TimerEvent &event);

    ros::Timer ModelSwitchTimer;
    void ModelSwitchTimerCallBack(const ros::TimerEvent &event);

    std::mutex FireStateMutex_;
    std::mutex UpdateMovePlannerStatusMutex_;
};

#endif /* FIRE_NAV_H_ */
