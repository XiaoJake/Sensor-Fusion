/*
 * @Description: 前端里程计算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:53:06
 */
#include "lidar_localization/front_end/front_end.hpp"

#include <fstream>
#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
FrontEnd::FrontEnd()
    :local_map_ptr_(new CloudData::CLOUD()),
     global_map_ptr_(new CloudData::CLOUD()),
     result_cloud_ptr_(new CloudData::CLOUD()),
     result_reflector_ptr_(new CloudData::CLOUD()) {

    InitWithConfig();
}

bool FrontEnd::InitWithConfig() {
    std::string config_file_path = WORK_SPACE_PATH + "/config/front_end/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    InitDataPath(config_node);
    InitRegistration(registration_ptr_, config_node);
    InitFilter("local_map", local_map_filter_ptr_, config_node);
    InitFilter("frame", frame_filter_ptr_, config_node);
    InitFilter("display", display_filter_ptr_, config_node);

    return true;
}

bool FrontEnd::InitParam(const YAML::Node& config_node) {
    key_frame_distance_ = config_node["key_frame_distance"].as<float>();
    local_frame_num_ = config_node["local_frame_num"].as<int>();

    return true;
}

bool FrontEnd::InitDataPath(const YAML::Node& config_node) {
    data_path_ = config_node["data_path"].as<std::string>();
    if (data_path_ == "./") {
        data_path_ = WORK_SPACE_PATH;
    }
    data_path_ += "/slam_data";

    if (boost::filesystem::is_directory(data_path_)) {
        boost::filesystem::remove_all(data_path_);
    }

    boost::filesystem::create_directory(data_path_);
    if (!boost::filesystem::is_directory(data_path_)) {
        LOG(WARNING) << "Cannot create directory " << data_path_ << "!";
        return false;
    } else {
        LOG_EVERY_N(INFO,10) << "Point Cloud Map Output Path: " << data_path_;
    }

    std::string key_frame_path = data_path_ + "/key_frames";
    boost::filesystem::create_directory(data_path_ + "/key_frames");
    if (!boost::filesystem::is_directory(key_frame_path)) {
        LOG(WARNING) << "Cannot create directory " << key_frame_path << "!";
        return false;
    } else {
        LOG_EVERY_N(INFO,10) << "Key Frames Output Path: " << key_frame_path << std::endl << std::endl;
    }

    return true;
}

bool FrontEnd::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node) {
    std::string registration_method = config_node["registration_method"].as<std::string>();
    LOG_EVERY_N(INFO,10) << "Point Cloud Registration Method: " << registration_method;

    if (registration_method == "NDT") {
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    } else if (registration_method == "ICP") {
        registration_ptr = std::make_shared<ICPRegistration>(config_node[registration_method]);
    } else if (registration_method == "ICP_SVD") {
        registration_ptr = std::make_shared<ICPSVDRegistration>(config_node[registration_method]);
    } else if (registration_method == "ICP_GN") {
        registration_ptr = std::make_shared<ICPGNRegistration>(config_node[registration_method]);
    } else {
        LOG(ERROR) << "Point cloud registration method " << registration_method << " NOT FOUND!";
        return false;
    }

    return true;
}

bool FrontEnd::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();
    LOG_EVERY_N(INFO,10) << filter_user << "Point Cloud Filter Method: " << filter_mothod;

    if (filter_mothod == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
    } else {
        LOG(ERROR) << "Point cloud filter method " << filter_mothod << " NOT FOUND!";
        return false;
    }

    return true;
}

bool FrontEnd::UpdateReflectorLoc(CloudData::CLOUDI_PTR points_raw, Eigen::Matrix4f& reflector_pose) {
    CloudData::CLOUDI_PTR reflector_points_i(new CloudData::CLOUDI);
    CloudData::CLOUD::Ptr reflector_points(new CloudData::CLOUD);
    CloudData::POINT_POSE centers;
    msg_reflector;

    auto point_transformed_to_base_link = [&](const Eigen::Vector2f& p) -> Eigen::Vector2f{
        const float x = p.x() * std::cos(lidar_to_base_.z()) - p.y() * std::sin(lidar_to_base_.z()) + lidar_to_base_.x();
        const float y = p.x() * std::sin(lidar_to_base_.z()) + p.y() * std::cos(lidar_to_base_.z()) + lidar_to_base_.y();
        return Eigen::Vector2f(x,y);
    };

    for(int i = 0; i < points_raw->points.size(); i++)
    {
        if(points_raw->at(i).intensity > intensity_min_)
        {
            reflector_points_i->points.push_back(points_raw->at(i));
        }
    }

    pcl::copyPointCloud(*reflector_points_i , *reflector_points);

    // 离群点剔除
    // 1)统计滤波法
    pcl::StatisticalOutlierRemoval<CloudData::CLOUD> sor;
    sor.setInputCloud(reflector_points);
    sor.setMeanK(30);//临近点
    sor.setStddevMulThresh(0.5);//距离大于1倍标准方差，值越大，丢掉的点越少
    sor.filter(*result_reflector_ptr_);

    // 2)半径滤波法
    // pcl::RadiusOutlierRemoval<CloudData::CLOUD> outrem;
    // outrem.setInputCloud(reflector_points);
    // outrem.setRadiusSearch(0.3);
    // outrem.setMinNeighborsInRadius(4);
    // outrem.filter(*result_reflector_ptr_);

    // 取反，获取被剔除的离群点
    // sor.setNegative(true);
    // sor.filter(*cloud_filtered);

    // 通过聚类，提取出反光板点云块
    // 创建一个Kd树对象作为提取点云时所用的方法，
    pcl::search::KdTree<CloudData::CLOUD>::Ptr tree(new pcl::search::KdTree<CloudData::CLOUD>);
    tree->setInputCloud (result_reflector_ptr_);//创建点云索引向量，用于存储实际的点云信息
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<CloudData::CLOUD> ec;//欧式聚类对象
    ec.setClusterTolerance(0.2); //设置近邻搜索的搜索半径(m)
    ec.setMinClusterSize(4);//设置一个聚类需要的最少点数目
    ec.setMaxClusterSize(160); //设置一个聚类需要的最大点数目
    ec.setSearchMethod(tree);//设置点云的搜索机制
    ec.setInputCloud(result_reflector_ptr_);
    ec.extract(cluster_indices);//从点云中提取聚类，并将 聚类后的点云块索引 保存在cluster_indices中

    //迭代访问点云索引cluster_indices，直到分割出所有聚类出的反光板点云
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        CloudData::CLOUD_PTR cloud_cluster (new CloudData::CLOUD);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back(result_reflector_ptr_->points[*pit]);

        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        cloud_cluster->header = result_reflector_ptr_->header;
        cloud_cluster->sensor_orientation_ = result_reflector_ptr_->sensor_orientation_;
        cloud_cluster->sensor_origin_ = result_reflector_ptr_->sensor_origin_;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

        // PCL函数计算质心
        Eigen::Vector4f centroid;// 质量m默认为1 (x,y,z,1)
        pcl::compute3DCentroid(*cloud_cluster, centroid);	// 计算当前质心
        Eigen::Vector2f now_point{centroid(0),centroid(1)};
        centers.push_back(point_transformed_to_base_link(now_point));
    }

    if(centers.empty())
        return false;
    // 输出检测到的反光板个数
    std::cout << "\n detected " << centers.size() << " reflectors\n";
    return true;
}


bool FrontEnd::Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose) {
    current_frame_.cloud_data.time = cloud_data.time;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *current_frame_.cloud_data.cloud_ptr, indices);

    // 对当前帧的点云进行体素滤波,完成下采样,减少匹配时的数据量
    CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
    frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr, filtered_cloud_ptr);

    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = init_pose_;

    // 局部地图容器中没有关键帧，代表是第一帧数据
    // 此时把当前帧数据作为第一个关键帧，并更新局部地图容器和全局地图容器
    if (local_map_frames_.size() == 0) {
        current_frame_.pose = init_pose_;// 完成第一帧点云的 位姿初始化
        UpdateWithNewFrame(current_frame_);
        cloud_pose = current_frame_.pose;
        return true;
    }

    // 不是第一帧，就正常匹配
    registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr_, current_frame_.pose);
    //假定机器人只在水平面运动,只保留yaw角的转动
    current_frame_.pose(0,2) = 0;current_frame_.pose(1,2) = 0;current_frame_.pose(2,2) = 1;
    current_frame_.pose(2,0) = 0;current_frame_.pose(2,1) = 0;
    //将z固定在水平面
    current_frame_.pose(2,3) = 0;

    cloud_pose = current_frame_.pose;//匹配后的位姿就是 当前帧的位姿

    // 更新相邻两帧的相对运动。采用匀速运动模型进行位姿预测:上一帧到当前帧的移动位姿=当前帧到下一阵的移动位姿
    step_pose = last_pose.inverse() * current_frame_.pose;// 当前帧位姿变换矩阵 左乘 上一时刻位姿的逆 = 相对于上一时刻的移动步长
    predict_pose = current_frame_.pose * step_pose;// 当前帧位姿变换矩阵 右乘 移动步长 = 预测的下一帧位姿
    last_pose = current_frame_.pose;

    // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新
    if (fabs(last_key_frame_pose(0,3) - current_frame_.pose(0,3)) +
        fabs(last_key_frame_pose(1,3) - current_frame_.pose(1,3)) +
        fabs(last_key_frame_pose(2,3) - current_frame_.pose(2,3)) > key_frame_distance_) {
        UpdateWithNewFrame(current_frame_);// 大于设定的生成关键帧的距离时,将当前帧作为关键帧存入,通过这些关键帧来拼接我们的 局部地图
        last_key_frame_pose = current_frame_.pose;
    }

    return true;
}

bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose) {
    init_pose_ = init_pose;
    return true;
}

bool FrontEnd::UpdateWithNewFrame(const Frame& new_key_frame) {
    // 把关键帧点云存储到硬盘里，节省内存
    std::string file_path = data_path_ + "/key_frames/key_frame_" + std::to_string(global_map_frames_.size()) + ".pcd";
    pcl::io::savePCDFileBinary(file_path, *new_key_frame.cloud_data.cloud_ptr);

    Frame key_frame = new_key_frame;
    // 这一步的目的是为了把关键帧的点云保存下来
    // 由于用的是共享指针，所以直接复制只是复制了一个指针而已
    // 此时无论你放多少个关键帧在容器里，这些关键帧点云指针都是指向的同一个点云
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));
    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());

    // 更新局部地图:如果此时局部地图中的 数量大于了 定义的最大局部地图存储数量local_frame_num_,
    // 就把最早的 局部地图弹出,让最新的一帧进来,这就是所谓的 滑窗
    local_map_frames_.push_back(key_frame);
    while (local_map_frames_.size() > static_cast<size_t>(local_frame_num_)) {
        local_map_frames_.pop_front();
    }

    local_map_ptr_.reset(new CloudData::CLOUD());
    for (size_t i = 0; i < local_map_frames_.size(); ++i) {//遍历滑窗
        // 点云位姿变换 参数：源点云，变换后的点云，变换矩阵(取自当前关键帧)
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr,
                                 *transformed_cloud_ptr,
                                 local_map_frames_.at(i).pose);

        *local_map_ptr_ += *transformed_cloud_ptr;// 将做好位姿变换的局部地图帧 存储到局部地图容器里
    }
    has_new_local_map_ = true;

    // 更新ndt匹配的目标点云
    // 关键帧数量还比较少的时候不滤波，因为点云本来就不多，太稀疏影响匹配效果
    if (local_map_frames_.size() < local_frame_num_) {
        registration_ptr_->SetInputTarget(local_map_ptr_);
    } else {
        CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
        local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);// 对局部地图也进行体素滤波,降低数据量
        // 设定NDT匹配中的 目标点云
        // registration_ptr_->ScanMatch就是把当前帧点云往这个 目标点云上匹配，得出的转换矩阵就需要求得的 位姿
        registration_ptr_->SetInputTarget(filtered_local_map_ptr);
    }

    // 保存所有关键帧信息在容器里
    // 存储之前，点云要先释放，因为已经存到了硬盘里，不释放也达不到节省内存的目的
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD());
    global_map_frames_.push_back(key_frame);

    return true;
}

bool FrontEnd::SaveMap() {
    global_map_ptr_.reset(new CloudData::CLOUD());

    std::string key_frame_path = "";
    CloudData::CLOUD_PTR key_frame_cloud_ptr(new CloudData::CLOUD());
    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());

    for (size_t i = 0; i < global_map_frames_.size(); ++i) {
        key_frame_path = data_path_ + "/key_frames/key_frame_" + std::to_string(i) + ".pcd";
        pcl::io::loadPCDFile(key_frame_path, *key_frame_cloud_ptr);

        pcl::transformPointCloud(*key_frame_cloud_ptr,
                                *transformed_cloud_ptr,
                                global_map_frames_.at(i).pose);
        *global_map_ptr_ += *transformed_cloud_ptr;
    }

    std::string map_file_path = data_path_ + "/map.pcd";
    pcl::io::savePCDFileBinary(map_file_path, *global_map_ptr_);
    has_new_global_map_ = true;

    return true;
}

bool FrontEnd::GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr) {
    if (has_new_local_map_) {
        display_filter_ptr_->Filter(local_map_ptr_, local_map_ptr);
        return true;
    }
    return false;
}

bool FrontEnd::GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr) {
    if (has_new_global_map_) {
        has_new_global_map_ = false;
        display_filter_ptr_->Filter(global_map_ptr_, global_map_ptr);
        global_map_ptr_.reset(new CloudData::CLOUD());
        return true;
    }
    return false;
}

bool FrontEnd::GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr) {
    display_filter_ptr_->Filter(result_cloud_ptr_, current_scan_ptr);
    return true;
}

bool FrontEnd::GetCurrentReflector(CloudData::CLOUD_PTR& current_reflector_ptr) {
    display_filter_ptr_->Show(result_reflector_ptr_, current_reflector_ptr);
    return true;
}
}