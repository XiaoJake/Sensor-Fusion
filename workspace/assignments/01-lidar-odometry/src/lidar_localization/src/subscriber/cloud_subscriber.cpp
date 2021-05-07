/*
 * @Description: 订阅激光点云信息，并解析数据
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */

#include "lidar_localization/subscriber/cloud_subscriber.hpp"

#include "glog/logging.h"

namespace lidar_localization {
CloudSubscriber::CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this);
}

void CloudSubscriber::msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr) {
    buff_mutex_.lock();
    static unsigned int frames_count = 0;
    static char hz_control = 2;
    frames_count++;

    if(frames_count%hz_control == 0)
    {
        CloudData cloud_data;
        cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
        pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));
        pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloudi_ptr));

        new_cloud_data_.push_back(cloud_data);
        //LOG_EVERY_N(INFO,10) << "got new pointclouds\n";
    }
    buff_mutex_.unlock();
}

void CloudSubscriber::ParseData(std::deque<CloudData>& cloud_data_buff, double time_calibration) {
    buff_mutex_.lock();

    if (new_cloud_data_.size() > 0) {
        cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end());
        new_cloud_data_.clear();

        // 把系统等待校准外参这段时间内接收的数据都 丢掉,只取收到校准外参后开始的数据
        while(cloud_data_buff.front().time <= time_calibration && cloud_data_buff.size() > 1)
            cloud_data_buff.pop_front();
    }

    buff_mutex_.unlock();
}
} // namespace data_input