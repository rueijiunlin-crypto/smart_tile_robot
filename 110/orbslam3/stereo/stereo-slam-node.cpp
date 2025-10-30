#include "stereo-slam-node.hpp"
#include <opencv2/core/core.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

// Helper function to convert ROS2 time to seconds
double StampToSec(const rclcpp::Time &stamp) {
    return stamp.seconds();  // Replace with appropriate method if necessary
}

StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM, const std::string &strSettingsFile, const std::string &strDoRectify)
    : Node("ORB_SLAM3_ROS2"), m_SLAM(pSLAM)
{
    std::stringstream ss(strDoRectify);
    ss >> std::boolalpha >> doRectify;

    if (doRectify) {
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened()) {
            std::cerr << "ERROR: Wrong path to settings" << std::endl;
            throw std::runtime_error("Wrong path to settings");
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;
        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;
        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;
        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0) {
            std::cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << std::endl;
            throw std::runtime_error("Calibration parameters missing");
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l, M2l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r, M2r);
    }

    left_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "/tile_robot/D435i/infra1/image_rect_raw");
    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "/tile_robot/D435i/infra2/image_rect_raw");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(
        approximate_sync_policy(10));
    syncApproximate->connectInput(*left_sub, *right_sub);
    syncApproximate->registerCallback(std::bind(&StereoSlamNode::GrabStereo, this, std::placeholders::_1, std::placeholders::_2));

    pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("camera/pose", 10);
}

StereoSlamNode::~StereoSlamNode()
{
    m_SLAM->Shutdown();
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoSlamNode::GrabStereo(const ImageMsg::ConstSharedPtr msgLeft, const ImageMsg::ConstSharedPtr msgRight)
{
    try {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    try {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    if (doRectify) {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image, imLeft, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image, imRight, M1r, M2r, cv::INTER_LINEAR);
        m_SLAM->TrackStereo(imLeft, imRight, StampToSec(msgLeft->header.stamp));
    } else {
        m_SLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, StampToSec(msgLeft->header.stamp));
    }

    PublishPose();
}

void StereoSlamNode::PublishPose()
{
    int trackingState = m_SLAM->GetTrackingState();
    if (trackingState == ORB_SLAM3::Tracking::OK || trackingState == ORB_SLAM3::Tracking::LOST) {
        cv::Mat imLeft = cv_ptrLeft->image;
        cv::Mat imRight = cv_ptrRight->image;
        double timestamp = StampToSec(cv_ptrLeft->header.stamp);

        Sophus::SE3f Tcw = m_SLAM->TrackStereo(imLeft, imRight, timestamp);

        cv::Mat Tcw_mat = cv::Mat::eye(4, 4, CV_32F);
        Tcw_mat.at<float>(0, 3) = Tcw.translation()(0);
        Tcw_mat.at<float>(1, 3) = Tcw.translation()(1);
        Tcw_mat.at<float>(2, 3) = Tcw.translation()(2);

        Eigen::Matrix3f rotation_matrix = Tcw.rotationMatrix();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                Tcw_mat.at<float>(i, j) = rotation_matrix(i, j);
            }
        }

        if (!Tcw_mat.empty()) {
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = this->now();
            pose_msg.header.frame_id = "map";

            // Inverting x and y as per the new coordinate system
            pose_msg.pose.position.x = -Tcw_mat.at<float>(0, 3);  // x axis: right negative, left positive
            pose_msg.pose.position.y = -Tcw_mat.at<float>(1, 3);  // y axis: down negative, up positive
            pose_msg.pose.position.z = Tcw_mat.at<float>(2, 3);

            cv::Mat Rcw = Tcw_mat(cv::Range(0, 3), cv::Range(0, 3));
            cv::Mat Rcw_double;
            Rcw.convertTo(Rcw_double, CV_64F);

            tf2::Matrix3x3 tf2_Rcw;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    tf2_Rcw[i][j] = Rcw_double.at<double>(i, j);
                }
            }

            tf2::Quaternion q;
            tf2_Rcw.getRotation(q);
            pose_msg.pose.orientation.x = q.x();
            pose_msg.pose.orientation.y = q.y();
            pose_msg.pose.orientation.z = q.z();
            pose_msg.pose.orientation.w = q.w();

            pose_publisher->publish(pose_msg);
        }
    }
}
