#ifndef __STEREO_SLAM_NODE_HPP__
#define __STEREO_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <cv_bridge/cv_bridge.h>
#include "System.h"

class StereoSlamNode : public rclcpp::Node
{
public:
    StereoSlamNode(ORB_SLAM3::System* pSLAM, const std::string &strSettingsFile, const std::string &strDoRectify);
    ~StereoSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;

    // Define the synchronizer policy with only two messages (left and right images)
    using approximate_sync_policy = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;

    void GrabStereo(const ImageMsg::ConstSharedPtr msgLeft, const ImageMsg::ConstSharedPtr msgRight);
    void PublishPose();

    ORB_SLAM3::System* m_SLAM;
    bool doRectify;
    cv::Mat M1l, M2l, M1r, M2r;

    cv_bridge::CvImageConstPtr cv_ptrLeft;
    cv_bridge::CvImageConstPtr cv_ptrRight;

    std::shared_ptr<message_filters::Subscriber<ImageMsg>> left_sub;
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> right_sub;
    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
};

#endif  // __STEREO_SLAM_NODE_HPP__
