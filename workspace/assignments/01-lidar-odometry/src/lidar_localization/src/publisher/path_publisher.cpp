/*
 * @Description: 里程计信息发布
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:11:44
 */
#include "lidar_localization/publisher/path_publisher.hpp"

namespace lidar_localization {
PathPublisher::PathPublisher(ros::NodeHandle& nh,
                                     std::string topic_name,
                                     std::string base_frame_id,
                                     int buff_size)
    :nh_(nh) {

    publisher_ = nh_.advertise<nav_msgs::Path>(topic_name, buff_size);
    path_.header.frame_id = base_frame_id;
}

void PathPublisher::Publish(const Eigen::Matrix4f& transform_matrix) {
    path_.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose;

    //set the position
    pose.pose.position.x = transform_matrix(0,3);
    pose.pose.position.y = transform_matrix(1,3);
    pose.pose.position.z = transform_matrix(2,3);

    Eigen::Quaternionf q;
    q = transform_matrix.block<3,3>(0,0);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    path_.poses.push_back(pose);
    publisher_.publish(path_);
}
}