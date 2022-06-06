#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ExperNodeBase.hpp>
#include <signal.h>

#define MACRO_ODOM_TOPIC "/odometry"
#define MACRO_TF_TOPIC "/transform"

class CovOdom2Tf : public ExperNodeBase
{
public:
    CovOdom2Tf(int nArgc, char **ppcArgv, const char *pcNodeName)
        : ExperNodeBase(nArgc, ppcArgv, pcNodeName)
    {
        //设置ODOM接收器
        mSubOdom = mupNodeHandle->subscribe(MACRO_ODOM_TOPIC, 1, &CovOdom2Tf::call_back, this);
        //设置TF发布器
        mPubTransform = mupNodeHandle->advertise<geometry_msgs::TransformStamped>(MACRO_TF_TOPIC, 1);
    }

    /* 析构函数 */
    ~CovOdom2Tf(){};
    // 主循环
    void Run(void) override
    {
        mbQuit = false;
        while (!mbQuit)
        {
            // 订阅节点
            ros::spinOnce();
            // ros::Duration(1).sleep();
        }
    }

    void call_back(const nav_msgs::OdometryConstPtr &msg)
    {
        // odometry_ = msg;
        odometry_.header = msg->header;
        odometry_.child_frame_id = msg->child_frame_id;
        odometry_.pose = msg->pose;
        odometry_.twist = msg->twist;

        geometry_msgs::TransformStamped tf_stamped;
        tf_stamped.header.frame_id = "/world";
        tf_stamped.header = odometry_.header;
        tf_stamped.child_frame_id = odometry_.child_frame_id;
        tf_stamped.transform.translation.x = odometry_.pose.pose.position.x;
        tf_stamped.transform.translation.y = odometry_.pose.pose.position.y;
        tf_stamped.transform.translation.z = odometry_.pose.pose.position.z;
        tf_stamped.transform.rotation = odometry_.pose.pose.orientation;
        if (tf_stamped.transform.rotation.x != 0 && tf_stamped.transform.rotation.y != 0 && tf_stamped.transform.rotation.z != 0 && tf_stamped.transform.rotation.w != 0)
        {
            mPubTransform.publish(tf_stamped);
        }
    }

    static bool mbQuit;

    static void OnSignalInterrupt(int nSigId)
    {
        ROS_WARN("Ctrl+C Pressed, program terminated.");
        mbQuit = true;
    }

private:
    ros::Subscriber mSubOdom;     //订阅器
    ros::Publisher mPubTransform; //发布器

    nav_msgs::Odometry odometry_; //接受的的odom话题消息
};

bool CovOdom2Tf::mbQuit;

main(int argc, char **argv)
{
    CovOdom2Tf node(argc, argv, "convert_odometry_to_transform");
    signal(SIGINT, &CovOdom2Tf::OnSignalInterrupt); //检测强制退出
    node.Run();                                     // 运行
    return 0;
}