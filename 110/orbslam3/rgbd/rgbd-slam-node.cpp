#include "rgbd-slam-node.hpp"
#include <opencv2/core/core.hpp>
#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/tile_robot/D435i/color/image_raw");
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/tile_robot/D435i/aligned_depth_to_color/image_raw");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);

    // Initialize the pose publisher
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/camera/pose", 10);
}

RgbdSlamNode::~RgbdSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void RgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Track RGBD data using ORB_SLAM3 and get the pose
    Sophus::SE3f Tcw = m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(msgRGB->header.stamp));

    // Extract the rotation and translation from Tcw (Camera-to-World)
    Eigen::Matrix4f pose_matrix = Tcw.matrix();

    Eigen::Vector3f translation = pose_matrix.block<3, 1>(0, 3); // x, y, z translation
    Eigen::Matrix3f rotation_matrix = pose_matrix.block<3, 3>(0, 0); // 3x3 rotation matrix

    // Convert rotation matrix to quaternion
    Eigen::Quaternionf quaternion(rotation_matrix);

    // Create PoseStamped message
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = msgRGB->header.stamp;
    pose_msg.header.frame_id = "map";

    // Set position (x, y, z)
    pose_msg.pose.position.x = translation.x();
    pose_msg.pose.position.y = translation.y();
    pose_msg.pose.position.z = translation.z();

    // Set orientation (quaternion)
    pose_msg.pose.orientation.x = quaternion.x();
    pose_msg.pose.orientation.y = quaternion.y();
    pose_msg.pose.orientation.z = quaternion.z();
    pose_msg.pose.orientation.w = quaternion.w();

    // Publish the pose message
    pose_pub_->publish(pose_msg);
}


