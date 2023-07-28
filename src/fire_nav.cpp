#include <fire_nav/fire_nav.h>

FireManager::FireManager(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    ROS_INFO("Hello, fire_nav.");
    initialize();
    setupTimer();
}

void FireManager::initialize()
{
    ROS_INFO("Register zmq.");
    ContextPtr_.reset(new zmq::context_t(1));
    ReqSockPtr_.reset(new zmq::socket_t(*ContextPtr_, ZMQ_REQ));

    ReqSockPtr_->setsockopt(ZMQ_IDENTITY, "client1", 7);
    ReqSockPtr_->connect("tcp://localhost:7001");
    ROS_INFO("Register zmq succeed.");
    IdSeqPub_ = 1;
    std::string NavTarget_topic_, NavStatus_topic_;

    nh_.param<std::string>("/fire_nav/fire_nav_settings/WorldFrame", WorldFrame_,
                           std::string("map"));
    nh_.param<std::string>("/fire_nav/fire_nav_settings/BodyFrame", BodyFrame_,
                           std::string("base_link"));
    nh_.param<std::string>("/fire_nav/fire_nav_settings/CameraFrame", CameraFrame_,
                           std::string("Camera"));
    nh_.param<std::string>("/fire_nav/fire_nav_settings/TargetFrame", TargetFrame_,
                           std::string("Fire"));

    nh_.param<std::string>("/fire_nav/fire_nav_settings/NavTarget_topic", NavTarget_topic_,
                           std::string("/move_base_simple/goal"));

    PlannerTargetPub_ = nh_.advertise<geometry_msgs::PoseStamped>(NavTarget_topic_.c_str(), 10);

    nh_.param<std::string>("/fire_nav/fire_nav_settings/NavStatus_topic", NavStatus_topic_,
                           std::string("/hello"));
    NavStatusSubscriber_ = nh_.subscribe(NavStatus_topic_.c_str(), 1,
                                         &FireManager::NavStatusCallBack, this);
}

void FireManager::setupTimer()
{
    if (1)
    {
        double UpdateFireFps_;
        nh_.param<double>("/fire_nav/fire_nav_settings/UpdateFireFps", UpdateFireFps_, 1.0);
        double duration = 1.0 / (UpdateFireFps_ + 0.00001);
        ReceiveAndUpdateFireStateTimer = nh_.createTimer(ros::Duration(duration),
                                                         &FireManager::ReceiveAndUpdateFireStateTimerCallBack, this);
    }
    if (0)
    {
        double ModelSwitch_Fps_;
        nh_.param<double>("/nav_upper/nav_manager_settings/ModelSwitch_Fps", ModelSwitch_Fps_, 3.0);
        double duration_ModelSwitch = 1.0 / (ModelSwitch_Fps_ + 0.00001);
        ModelSwitchTimer = nh_.createTimer(ros::Duration(duration_ModelSwitch),
                                           &FireManager::ModelSwitchTimerCallBack, this);
    }
}

void FireManager::NavStatusCallBack(const actionlib_msgs::GoalStatusArray &msg)
{
    std::lock_guard<std::mutex> lock3(UpdateMovePlannerStatusMutex_);
    if (msg.status_list.size() > 0)
    {
        MoveBaseStatus_ = msg.status_list[msg.status_list.size() - 1].status;
    }
}

void FireManager::ReceiveAndUpdateFireStateTimerCallBack(const ros::TimerEvent &event)
{
    std::lock_guard<std::mutex> lock1(FireStateMutex_);

    ROS_INFO("hello timer");

    // zmq::context_t Context_(1);
    // zmq::socket_t ReqSock(Context_,ZMQ_DEALER);
    // ReqSock.setsockopt(ZMQ_IDENTITY, "client1", 7);
    // ReqSock.connect("tcp://localhost:7001");
    //  tf::TransformListener world_base_listener;
    //  tf::StampedTransform world_base_transform;
    //  try
    //  {
    //      world_base_listener.waitForTransform(WorldFrame_.c_str(), BodyFrame_.c_str(), ros::Time(0), ros::Duration(1.0));
    //      world_base_listener.lookupTransform(WorldFrame_.c_str(), BodyFrame_.c_str(),
    //                                          ros::Time(0), world_base_transform);
    //  }
    //  catch (tf::TransformException &ex)
    //  {
    //      ROS_ERROR("There is something wrong when trying get robot pose to get target orientation.");
    //      ROS_ERROR("%s", ex.what());
    //      // ros::Duration(1.0).sleep();
    //  }

    // tf::TransformListener world_camera_listener;
    // tf::StampedTransform world_camera_transform;
    // try
    // {
    //     world_camera_listener.waitForTransform(WorldFrame_.c_str(), CameraFrame_.c_str(), ros::Time(0), ros::Duration(1.0));
    //     world_camera_listener.lookupTransform(WorldFrame_.c_str(), CameraFrame_.c_str(),
    //                                           ros::Time(0), world_camera_transform);
    // }
    // catch (tf::TransformException &ex)
    // {
    //     ROS_ERROR("There is something wrong when trying get camera pose to get target orientation.");
    //     ROS_ERROR("%s", ex.what());
    //     // ros::Duration(1.0).sleep();
    // }

    zmq_msg_t msgIn;
    zmq_msg_init(&msgIn);
    zmq_msg_recv(&msgIn, *ReqSockPtr_, 0);
    std::cout << "recv:" << (char *)zmq_msg_data(&msgIn) << std::endl;
    zmq_msg_close(&msgIn);
    ROS_INFO("end receive");
    // TODO Receive message.
    tf::Quaternion quat;
    quat.setEuler(0.0, 0.0, 0.0);
    tf::Vector3 origin(0.0, 0.0, 0.0);
    tf::Pose poseTF(quat, origin);

    tf::Transform CameraToTargetTransformMiddle;
    CameraToTargetTransformMiddle.setOrigin(tf::Vector3(0.0, 2.0, 0.0));
    CameraToTargetTransformMiddle.setRotation(tf::Quaternion(0, 0, 0, 1));
    brCTT.sendTransform(tf::StampedTransform(CameraToTargetTransformMiddle,
                                             ros::Time::now(),
                                             CameraFrame_.c_str(),
                                             TargetFrame_.c_str()));

    tf::TransformListener world_target_listener;
    tf::StampedTransform world_target_transform;
    try
    {
        world_target_listener.waitForTransform(WorldFrame_.c_str(), CameraFrame_.c_str(), ros::Time(0), ros::Duration(1.0));
        world_target_listener.lookupTransform(WorldFrame_.c_str(), CameraFrame_.c_str(),
                                              ros::Time(0), world_target_transform);
        /*
        world_target_listener.waitForTransform(WorldFrame_.c_str(), TargetFrame_.c_str(), ros::Time(0), ros::Duration(1.0));
                world_target_listener.lookupTransform(WorldFrame_.c_str(), TargetFrame_.c_str(),
                                                      ros::Time(0), world_target_transform);*/
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("There is something wrong when trying get target pose and get target orientation.");
        ROS_ERROR("%s", ex.what());
        // ros::Duration(1.0).sleep();
    }

    tf::Quaternion q_target = world_target_transform.getRotation();
    double yaw_target_ = tf::getYaw(q_target);
    geometry_msgs::PoseStamped TargetPose_;
    TargetPose_.header.frame_id = WorldFrame_.c_str();
    TargetPose_.header.seq = IdSeqPub_;
    IdSeqPub_++;
    TargetPose_.header.stamp = ros::Time::now();
    TargetPose_.pose.position.x = world_target_transform.getOrigin().x();
    TargetPose_.pose.position.y = world_target_transform.getOrigin().y();
    TargetPose_.pose.position.z = 0.0;
    tf2::Quaternion q_target_tf2_;
    q_target_tf2_.setRPY(0, 0, yaw_target_);
    q_target_tf2_.normalize();
    geometry_msgs::Quaternion q_target_msg_ = tf2::toMsg(q_target_tf2_);

    TargetPose_.pose.orientation = q_target_msg_;

    PlannerTargetPub_.publish(TargetPose_);
}

void FireManager::ModelSwitchTimerCallBack(const ros::TimerEvent &event)
{
    std::lock_guard<std::mutex> lock3(UpdateMovePlannerStatusMutex_);
}
