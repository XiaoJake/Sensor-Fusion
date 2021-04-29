/*
 * @Description:
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:17:49
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace lidar_localization {
class CloudData {
  public:
    using POINT = pcl::PointXYZ;
    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUD_PTR = CLOUD::Ptr;

    using POINTI = pcl::PointXYZI;
    using CLOUDI = pcl::PointCloud<POINTI>;
    using CLOUDI_PTR = CLOUDI::Ptr;
    using POINT_POSE = std::vector<Eigen::Vector2f>;

  public:
    CloudData()
      :cloud_ptr(new CLOUD()), cloudi_ptr(new CLOUDI()) {
    }

  public:
    double time = 0.0;
    CLOUD_PTR cloud_ptr;
    CLOUDI_PTR cloudi_ptr;
};
}

#endif