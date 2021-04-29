/*
 * @Description: front end 任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */
#include "lidar_localization/front_end/front_end_flow.hpp"

#include "glog/logging.h"

#include "lidar_localization/tools/file_manager.hpp"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh) {
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 50);
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 50);
    velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 50);
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 50);
    lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, "imu_link", "laser_link");

    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "current_scan", 10, "/map");
    reflector_pub_ptr = std::make_shared<CloudPublisher>(nh, "reflector", 10, "/map");
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "local_map", 10, "/map");
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "global_map", 10, "/map");
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "laser_odom", "map", "lidar", 10);
    gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "gnss", "map", "lidar", 10);

    front_end_ptr_ = std::make_shared<FrontEnd>();

    local_map_ptr_.reset(new CloudData::CLOUD());
    global_map_ptr_.reset(new CloudData::CLOUD());
    current_scan_ptr_.reset(new CloudData::CLOUD());
    current_reflector_ptr.reset(new CloudData::CLOUD());

    use_reflector_ = true;
}

bool FrontEndFlow::Run() {
    if (!InitCalibration())  {
        LOG_EVERY_N(INFO,10) << "Waiting for init calibration!\n";
        return false;
    }

    if(!use_reflector_)
        if (!InitGNSS()) {
            LOG_EVERY_N(INFO,10) << "Failed to init GNSS!\n";
            return false;
        }

    if (!ReadData()) {
        LOG_EVERY_N(INFO,10) << "Failed to read data!\n";
        return false;
    }

    while(HasData()) {
        if (!ValidData()) {
            LOG_EVERY_N(INFO,500) << "Invalid data!\n";
            continue;
        }

        //UpdateGNSSOdometry();
        UpdateReflectorOdometry();
        if (UpdateLaserOdometry()) {
            PublishData();
            SaveTrajectory();
            //LOG_EVERY_N(INFO,10) << "Update LaserOdometry...\n";
        } else {
            LOG_EVERY_N(INFO,10) << "UpdateLaserOdometry failed!\n";
        }
    }

    return true;
}

bool FrontEndFlow::ReadData() {
    //缓存点云数据
    cloud_sub_ptr_->ParseData(cloud_data_buff_, time_calibration_);

    static std::deque<IMUData> unsynced_imu_;
    static std::deque<VelocityData> unsynced_velocity_;
    static std::deque<GNSSData> unsynced_gnss_;

    bool valid_velocity;
    bool valid_gnss;

    //缓存imu数据(未时间同步)
    imu_sub_ptr_->ParseData(unsynced_imu_, time_calibration_);
    if(!use_reflector_)
    {
        velocity_sub_ptr_->ParseData(unsynced_velocity_);
        gnss_sub_ptr_->ParseData(unsynced_gnss_);
    }

    if (cloud_data_buff_.size() == 0) {
        LOG_EVERY_N(INFO,10) << "Waiting for laser scan..." << std::endl;
        return false;
    }

    // 将imu数据以点云数据时间戳为基准做 线性插值的时间同步
    double cloud_time = cloud_data_buff_.front().time;
    bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff_, cloud_time);
    if(!use_reflector_)
    {
        valid_velocity = VelocityData::SyncData(unsynced_velocity_, velocity_data_buff_, cloud_time);
        valid_gnss = GNSSData::SyncData(unsynced_gnss_, gnss_data_buff_, cloud_time);
    }

    static bool sensor_inited = false;
    if (!sensor_inited) {
        if(!use_reflector_)
        {
            if (!valid_imu || !valid_velocity || !valid_gnss) {
            LOG_EVERY_N(INFO,10) << "Validity check: \n"
                      << "IMU: " << valid_imu << ", "
                      << "Velocity: " << valid_velocity << ", "
                      << "GNSS: " << valid_gnss << "\n";
            cloud_data_buff_.pop_front();
            return false;
            }
        }
        else if (!valid_imu) {
            LOG_EVERY_N(INFO,10) << "Validity check: \n"
                                 << "IMU: " << valid_imu << "\n";
            cloud_data_buff_.pop_front();
            return false;
        }

        sensor_inited = true;
    }

    return true;
}

bool FrontEndFlow::InitCalibration() {
    static bool calibration_received = false;
    if (!calibration_received) {
        if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
            calibration_received = true;
            time_calibration_ = ros::Time::now().toSec();
            LOG_EVERY_N(INFO,10) << "OK! Succeed to init calibration!\n";
        }
    }

    return calibration_received;
}

bool FrontEndFlow::InitGNSS() {
    static bool gnss_inited = false;
    if (!gnss_inited) {
        GNSSData gnss_data = gnss_data_buff_.front();
        gnss_data.InitOriginPosition();
        gnss_inited = true;
    }

    return gnss_inited;
}

bool FrontEndFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    if (imu_data_buff_.size() == 0)
        return false;

    if(!use_reflector_)
    {
        if (velocity_data_buff_.size() == 0)
            return false;
        if (gnss_data_buff_.size() == 0)
            return false;
    }

    //LOG(INFO) << "points size:= " << cloud_data_buff_.size();
    //LOG(INFO) << "has data...";
    return true;
}

bool FrontEndFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    if(!use_reflector_)
    {
        current_velocity_data_ = velocity_data_buff_.front();
        current_gnss_data_ = gnss_data_buff_.front();
    }


    double d_time = current_cloud_data_.time - current_imu_data_.time;
    if (d_time < -0.05) {
        cloud_data_buff_.pop_front();
        LOG_EVERY_N(INFO,1) << "Points too old\n";
        return false;
    }

    if (d_time > 0.05) {
        imu_data_buff_.pop_front();
        if(!use_reflector_)
        {
            velocity_data_buff_.pop_front();
            gnss_data_buff_.pop_front();
        }
        LOG_EVERY_N(INFO,1) << "Other sensor's data too old\n";
        return false;
    }

    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    if(!use_reflector_)
    {
        velocity_data_buff_.pop_front();
        gnss_data_buff_.pop_front();
    }

    return true;
}

bool UpdateReflectorOdometry(){
    reflector_odometry_ = ;
    return true;
}

bool FrontEndFlow::UpdateGNSSOdometry() {
    current_gnss_data_.UpdateXYZ();
    gnss_odometry_(0,3) = current_gnss_data_.local_E;
    gnss_odometry_(1,3) = current_gnss_data_.local_N;
    gnss_odometry_(2,3) = current_gnss_data_.local_U;
    gnss_odometry_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix();
    gnss_odometry_ *= lidar_to_imu_;

    return true;
}

bool FrontEndFlow::UpdateLaserOdometry() {
    static bool front_end_pose_inited = false;
    if (!front_end_pose_inited) {
        front_end_pose_inited = true;
        if(!use_reflector_)
            front_end_ptr_->SetInitPose(gnss_odometry_);
        else
            front_end_ptr_->SetInitPose(Eigen::Matrix4f::Identity());

        return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
    }

    laser_odometry_ = Eigen::Matrix4f::Identity();
    return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
}

bool FrontEndFlow::PublishData() {
    gnss_pub_ptr_->Publish(gnss_odometry_);
    laser_odom_pub_ptr_->Publish(laser_odometry_);

    front_end_ptr_->GetCurrentScan(current_scan_ptr_);
    front_end_ptr_->GetCurrentReflector(current_reflector_ptr);
    cloud_pub_ptr_->Publish(current_scan_ptr_);
    reflector_pub_ptr->Publish(current_reflector_ptr);

    if (front_end_ptr_->GetNewLocalMap(local_map_ptr_))
        local_map_pub_ptr_->Publish(local_map_ptr_);

    return true;
}


bool FrontEndFlow::SaveTrajectory() {
    static std::ofstream ground_truth, laser_odom;
    static bool is_file_created = false;
    if (!is_file_created) {
        if (!FileManager::CreateDirectory(WORK_SPACE_PATH + "/slam_data/trajectory"))
            return false;
        if (!FileManager::CreateFile(ground_truth, WORK_SPACE_PATH + "/slam_data/trajectory/ground_truth.txt"))
            return false;
        if (!FileManager::CreateFile(laser_odom, WORK_SPACE_PATH + "/slam_data/trajectory/laser_odom.txt"))
            return false;
        is_file_created = true;
    }

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            ground_truth << gnss_odometry_(i, j);
            laser_odom << laser_odometry_(i, j);
            if (i == 2 && j == 3) {
                ground_truth << std::endl;
                laser_odom << std::endl;
            } else {
                ground_truth << " ";
                laser_odom << " ";
            }
        }
    }

    return true;
}

bool FrontEndFlow::SaveMap() {
    return front_end_ptr_->SaveMap();
}

bool FrontEndFlow::PublishGlobalMap() {
    if (front_end_ptr_->GetNewGlobalMap(global_map_ptr_)) {
        global_map_pub_ptr_->Publish(global_map_ptr_);
        global_map_ptr_.reset(new CloudData::CLOUD());
    }
    return true;
}
}