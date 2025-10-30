#include "monocular-slam-node.hpp"
#include <opencv2/core/core.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"), m_SLAM(pSLAM)
{
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "/tile_robot/D435i/color/image_raw",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    
    m_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("camera/pose", 10);
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    Sophus::SE3f Tcw = m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));

    // Publish the camera pose
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.header.frame_id = "world"; // Or use a different frame_id if needed

    // Extract position and orientation from Tcw
    Eigen::Matrix3f Rwc = Tcw.rotationMatrix().transpose();
    Eigen::Vector3f twc = -Rwc * Tcw.translation();

    pose_msg.pose.position.x = twc.x();
    pose_msg.pose.position.y = twc.y();
    pose_msg.pose.position.z = twc.z();

    tf2::Matrix3x3 tfRwc(
        Rwc(0, 0), Rwc(0, 1), Rwc(0, 2),
        Rwc(1, 0), Rwc(1, 1), Rwc(1, 2),
        Rwc(2, 0), Rwc(2, 1), Rwc(2, 2)
    );

    tf2::Quaternion q;
    tfRwc.getRotation(q);
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    m_pose_publisher->publish(pose_msg);
}
