/*
 * @Description: odometry 信息发布
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:05:47
 */
#ifndef LIDAR_LOCALIZATION_PUBLISHER_PATH_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_PATH_PUBLISHER_HPP_

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Path.h>

namespace lidar_localization {
class PathPublisher {
  public:
    PathPublisher(ros::NodeHandle& nh,
                      std::string topic_name,
                      std::string base_frame_id,
                      int buff_size);
    PathPublisher() = default;

    void Publish(const Eigen::Matrix4f& transform_matrix);

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    nav_msgs::Path path_;
};
}
#endif