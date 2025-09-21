#include "bag_tumrgbd/bagsub.h"

bagtumrgbd::bagtumrgbd(ros::NodeHandle& nh): it_(nh){
  p_pre.reset(new Preprocess());
  // p_pre.reset(new Preprocess());

  initializeparams(nh);

  initializefolder();

  set_img_dir_img_txt();

  set_dep_dir_dep_txt();

  set_imu_txt();

  set_mocap_txt();

  initializesubscriber(nh);
}

bagtumrgbd::~bagtumrgbd(){
  { std::lock_guard<std::mutex> lk(io_mutex_img); if (rgb_txt_.is_open()) rgb_txt_.close(); }
  { std::lock_guard<std::mutex> lk(io_mutex_imu); if (imu_txt_.is_open()) imu_txt_.close(); }
  { std::lock_guard<std::mutex> lk(io_mutex_mocap); if (mocap_txt_.is_open()) mocap_txt_.close(); }
  { std::lock_guard<std::mutex> lk(io_mutex_dep); if (depth_txt_.is_open()) depth_txt_.close(); }
}

void bagtumrgbd::initializeparams(ros::NodeHandle &nh){
  // nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
  // nh.param<string>("common/imu_topic", imu_topic, "/imu/data");

  nh.param<std::string>("common/img_topic", img_topic_, "/camera/image_color");
  nh.param<std::string>("common/imu_topic", imu_topic_, "/livox/imu");
  nh.param<std::string>("common/lid_topic", lid_topic_,"/livox/lidar");

  nh.param<bool>("gt/rtk", use_rtk, false);

  if(use_rtk){
    nh.param<std::string>("common/rtk_topic", rtk_topic_, "/dji_osdk_ros/rtk_position");
  }
  else{
    nh.param<std::string>("common/mocap_topic", mocap_topic_, "/mocap_node/track_1/pose");
  }

  std::cout << "===========================" << std::endl;
  std::cout << "           topics          " << std::endl;
  std::cout << "the subscribed topics are:" << std::endl;
  std::cout << "the img topic" << img_topic_ << std::endl;
  std::cout << "the imu topic" << imu_topic_ << std::endl;
  std::cout << "the lidar topic" << lid_topic_ << std::endl;
  if(use_rtk){
    std::cout << "the ground truth" << rtk_topic_ << std::endl;
  }
  else{
    std::cout << "the ground truth" << mocap_topic_ << std::endl;
  }
  std::cout << "===========================" << std::endl;

  // pcl preprocess
  nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
  nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
  nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 6);
  nh.param<int>("preprocess/point_filter_num", p_pre->point_filter_num, 3);
  nh.param<bool>("preprocess/feature_extract_enabled", p_pre->feature_enabled, false);

  std::cout << "===========================" << std::endl;
  std::cout << "         preprocess        " << std::endl;
  std::cout << "the lidar" << p_pre->lidar_type << std::endl;
  std::cout << "===========================" << std::endl;


  p_pre->blind_sqr = p_pre->blind * p_pre->blind;

  // Camera intrinsics
  nh.param("camera/width",  width,  1280);
  nh.param("camera/height", height, 720);
  nh.param("camera/fx", fx, 525.0);
  nh.param("camera/fy", fy, 525.0);
  nh.param("camera/cx", cx, 319.5);
  nh.param("camera/cy", cy, 239.5);


  // Depth range (m)
  nh.param("depth/min", depth_min_, 0.1);
  nh.param("depth/max", depth_max_, 60.0);

  std::cout << "===========================" << std::endl;
  std::cout << "      camera intrinsic     " << std::endl;
  std::cout << "width" << width << std::endl;
  std::cout << "height" << height << std::endl;
  std::cout << "fx" << fx << std::endl;
  std::cout << "fy" << fy << std::endl;
  std::cout << "cx" << cx << std::endl;
  std::cout << "cy" << cy << std::endl;
  std::cout << "depth min" << depth_min_ << std::endl;
  std::cout << "depth max" << depth_max_ << std::endl;
  std::cout << "===========================" << std::endl;

  // nh.param<vector<double>>("extrin_calib/extrinsic_T", extrinT, vector<double>());
  // nh.param<vector<double>>("extrin_calib/extrinsic_R", extrinR, vector<double>());
  nh.param<vector<double>>("extrin_calib/Pcl", cameraextrinT, vector<double>());
  nh.param<vector<double>>("extrin_calib/Rcl", cameraextrinR, vector<double>());

  // R_cl_ << MAT_FROM_ARRAY(cameraextrinR);
  // t_cl_ << VEC_FROM_ARRAY(cameraextrinT);

  if (cameraextrinR.size() == 9 && cameraextrinT.size() == 3) {
    R_cl_ << cameraextrinR[0], cameraextrinR[1], cameraextrinR[2],
            cameraextrinR[3], cameraextrinR[4], cameraextrinR[5],
            cameraextrinR[6], cameraextrinR[7], cameraextrinR[8];
    t_cl_ << cameraextrinT[0], cameraextrinT[1], cameraextrinT[2];
  } else {
    ROS_WARN("extrin_calib/Rcl or /Pcl missing or wrong size (got R:%zu, t:%zu). "
            "Using identity R and zero t. (Expect R:9, t:3)",
            cameraextrinR.size(), cameraextrinT.size());
  }
  std::cout << "===========================" << std::endl;
  std::cout << "       T_cl_extrinsic      " << std::endl;
  std::cout <<  "R_cl_" <<std::endl;
  std::cout <<  R_cl_ <<std::endl;
  std::cout <<  "t_cl_" <<std::endl;
  std::cout <<  t_cl_ <<std::endl;
  std::cout << "===========================" << std::endl;

}


std::string bagtumrgbd::guessPackageNameFromExe(){
  char buf[PATH_MAX];
  ssize_t len = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
  if (len <= 0) return {};
  buf[len] = '\0';
  boost::filesystem::path exe(buf);                    
  return exe.parent_path().filename().string();    
}

std::string bagtumrgbd::getPackagePathAuto(){
  #ifdef ROS_PACKAGE_NAME
    {
        std::string p = ros::package::getPath(ROS_PACKAGE_NAME);
        if (!p.empty()) return p;
    }
  #endif
  std::string pkg = guessPackageNameFromExe();
  if (!pkg.empty()) {
    std::string p = ros::package::getPath(pkg);
    if (!p.empty()) return p;
  }
  return {};
}

void bagtumrgbd::initializefolder(){
  const std::string pkg_path = getPackagePathAuto();
  if (!pkg_path.empty()) {
    out_dir = pkg_path + "/output";
  } else {
    ROS_WARN("Could not resolve package path automatically;");
  }

  if (!boost::filesystem::exists(out_dir)) boost::filesystem::create_directories(out_dir);
}

void bagtumrgbd::set_img_dir_img_txt(){
  rgb_dir_ = out_dir + "/rgb";
  
  try {
      if (!boost::filesystem::exists(out_dir)) boost::filesystem::create_directories(out_dir);
      if (!boost::filesystem::exists(rgb_dir_)) boost::filesystem::create_directories(rgb_dir_);
  } catch (const boost::filesystem::filesystem_error& e) {
      ROS_FATAL("Failed to create output directories under '%s': %s", out_dir.c_str(), e.what());
      throw;
  }

  // Open rgb.txt in append mode (TUM format, no header by request)
  rgb_txt_path_ = out_dir + "/rgb.txt";
  rgb_txt_.open(rgb_txt_path_, std::ios::out | std::ios::app);
  if (!rgb_txt_) {
      ROS_FATAL("Cannot open %s for writing.", rgb_txt_path_.c_str());
      throw std::runtime_error("open rgb.txt failed");
  }
  rgb_txt_.imbue(std::locale::classic());  // consistent decimal dot
  rgb_txt_ << std::fixed;                  // fixed-point for timestamps

  ROS_INFO("Images will be saved to: %s", rgb_dir_.c_str());
  ROS_INFO("Index file: %s", rgb_txt_path_.c_str());
}

void bagtumrgbd::set_imu_txt(){
  try {
      if (!boost::filesystem::exists(out_dir)) boost::filesystem::create_directories(out_dir);
      // if (!boost::filesystem::exists(rgb_dir_)) boost::filesystem::create_directories(rgb_dir_);
  } catch (const boost::filesystem::filesystem_error& e) {
      ROS_FATAL("Failed to create output directories under '%s': %s", out_dir.c_str(), e.what());
      throw;
  }
  imu_txt_path_ = out_dir + "/accelerometer.txt";
  // const bool need_header = !boost::filesystem::exists(imu_txt_path_);
  imu_txt_.open(imu_txt_path_, std::ios::out | std::ios::app);
  if (!imu_txt_) {
      ROS_FATAL("Cannot open %s for writing.", imu_txt_path_.c_str());
      throw std::runtime_error("open accelerometer.txt failed");
  }
  imu_txt_.imbue(std::locale::classic());  // consistent decimal dot
  imu_txt_ << std::fixed;                  // fixed-point for timestamps

  // if (need_header) {
  // std::lock_guard<std::mutex> lk(io_mutex_imu);
  // imu_txt_ << "# timestamp ax ay az\n";
  // imu_txt_.flush();
  // }

  ROS_INFO("accelerometer.txt set: %s", imu_txt_path_.c_str());
}

void bagtumrgbd::set_mocap_txt(){
  try 
  {
      if (!boost::filesystem::exists(out_dir)) boost::filesystem::create_directories(out_dir);
      // if (!boost::filesystem::exists(rgb_dir_)) boost::filesystem::create_directories(rgb_dir_);
  } 
  catch (const boost::filesystem::filesystem_error& e) 
  {
      ROS_FATAL("Failed to create output directories under '%s': %s", out_dir.c_str(), e.what());
      throw;
  }
  mocap_txt_path_ = out_dir + "/groundtruth.txt";
  // const bool need_header = !boost::filesystem::exists(imu_txt_path_);
  mocap_txt_.open(mocap_txt_path_, std::ios::out | std::ios::app);
  if (!mocap_txt_) {
      ROS_FATAL("Cannot open %s for writing.", mocap_txt_path_.c_str());
      throw std::runtime_error("open groundtruth.txt failed");
  }
  mocap_txt_.imbue(std::locale::classic());  // consistent decimal dot
  mocap_txt_ << std::fixed;                  // fixed-point for timestamps

  // if (need_header) {
  // std::lock_guard<std::mutex> lk(io_mutex_imu);
  // imu_txt_ << "# timestamp ax ay az\n";
  // imu_txt_.flush();
  // }

  ROS_INFO("groundtruth.txt set: %s", mocap_txt_path_.c_str());
}

void bagtumrgbd::set_dep_dir_dep_txt(){
  depth_dir_ = out_dir + "/depth";
  try {
      if (!boost::filesystem::exists(out_dir)) boost::filesystem::create_directories(out_dir);
      if (!boost::filesystem::exists(depth_dir_)) boost::filesystem::create_directories(depth_dir_);
  } catch (const boost::filesystem::filesystem_error& e) {
      ROS_FATAL("Failed to create output directories under '%s': %s", out_dir.c_str(), e.what());
      throw;
  }

  // Open rgb.txt in append mode (TUM format, no header by request)
  depth_txt_path_ = out_dir + "/rgb.txt";
  depth_txt_.open(depth_txt_path_, std::ios::out | std::ios::app);
  if (!depth_txt_) {
      ROS_FATAL("Cannot open %s for writing.", depth_txt_path_.c_str());
      throw std::runtime_error("open rgb.txt failed");
  }
  depth_txt_.imbue(std::locale::classic());  // consistent decimal dot
  depth_txt_ << std::fixed;                  // fixed-point for timestamps

  ROS_INFO("Images will be saved to: %s", depth_dir_.c_str());
  ROS_INFO("Index file: %s", depth_txt_path_.c_str());
}


void bagtumrgbd::initializesubscriber(ros::NodeHandle &nh){
    sub_imu = nh.subscribe(imu_topic_, 200000, &bagtumrgbd::imu_cbk, this);
    sub_img = it_.subscribe(img_topic_, 10, &bagtumrgbd::img_cbk, this);
    if(use_rtk){
      sub_rtk = nh.subscribe(rtk_topic_, 200000, &bagtumrgbd::rtk_cbk, this);
    }
    else{
      sub_mocap = nh.subscribe(mocap_topic_,1000,&bagtumrgbd::mocap_cbk,this);
    }

    sub_pcl = p_pre->lidar_type == AVIA ? 
            nh.subscribe(lid_topic_, 200000, &bagtumrgbd::livox_pcl_cbk, this): 
            nh.subscribe(lid_topic_, 200000, &bagtumrgbd::ac1_pcl_cbk, this);
    
    ROS_INFO("Subscribing to IMU topic:   %s", imu_topic_.c_str());
    ROS_INFO("Subscribing to image topic: %s", img_topic_.c_str());
    ROS_INFO("Subscribing to lidar topic: %s",lid_topic_.c_str());

    if(use_rtk){
      ROS_INFO("Subscribing to rtk topic: %s",rtk_topic_.c_str());
    }else{
      ROS_INFO("Subscribing to mocap topic: %s",mocap_topic_.c_str());
    }

}

void bagtumrgbd::img_cbk(const sensor_msgs::ImageConstPtr& msg){

    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    cv::Mat to_save;

    if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
      cv::cvtColor(cv_ptr->image, to_save, cv::COLOR_RGB2BGR);
    } else if (msg->encoding == sensor_msgs::image_encodings::BGR8 || msg->encoding == sensor_msgs::image_encodings::MONO8 || msg->encoding == sensor_msgs::image_encodings::MONO16) {
      to_save = cv_ptr->image;
    } else {
      // Convert other color encodings to BGR8 for portability
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      to_save = cv_ptr->image;
    }

    // Use the same timestamp string for both the left column and filename.
    std::ostringstream ts;
    ts.imbue(std::locale::classic());
    ts.setf(std::ios::fixed);
    ts << std::setprecision(6) << msg->header.stamp.toSec();  // TUM uses 6 decimals
    const std::string tstr = ts.str();

    const std::string filename = tstr + ".png";
    const std::string fullpath = rgb_dir_ + "/" + filename;

    if (!cv::imwrite(fullpath, to_save)) {
      ROS_ERROR("Failed to write image to %s", fullpath.c_str());
      return;
    }

    // Append to rgb.txt in a threadsafe way
    {
      std::lock_guard<std::mutex> lk(io_mutex_img);
      rgb_txt_ << tstr << " rgb/" << filename << '\n';
      rgb_txt_.flush();
    }

}

// IMU: write "# timestamp ax ay az" lines
void bagtumrgbd::imu_cbk(const sensor_msgs::ImuConstPtr& msg){
    const double t  = msg->header.stamp.toSec();
    const double ax = msg->linear_acceleration.x;
    const double ay = msg->linear_acceleration.y;
    const double az = msg->linear_acceleration.z;

    std::lock_guard<std::mutex> lk(io_mutex_imu);
    imu_txt_ << std::setprecision(6) << t << " "
            << std::setprecision(6) << ax << " "
            << std::setprecision(6) << ay << " "
            << std::setprecision(6) << az << "\n";
    imu_txt_.flush();
}

void bagtumrgbd::mocap_cbk(const geometry_msgs::PoseStampedConstPtr& msg){
    const double t  = msg->header.stamp.toSec();

    const double tx = msg->pose.position.x;
    const double ty = msg->pose.position.y;
    const double tz = msg->pose.position.z;

    const double qx = msg->pose.orientation.x;
    const double qy = msg->pose.orientation.y;
    const double qz = msg->pose.orientation.z;
    const double qw = msg->pose.orientation.w;

    std::lock_guard<std::mutex> lk(io_mutex_mocap);
    mocap_txt_ << std::fixed << std::setprecision(kGT_PREC)
            << t  << " "
            << tx << " " << ty << " " << tz << " "
            << qx << " " << qy << " " << qz << " " << qw << "\n";
    mocap_txt_.flush();
}

void bagtumrgbd::livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in){
  if (!lidar_en) return;
  // mtx_buffer.lock();
  livox_ros_driver::CustomMsg::Ptr msg(new livox_ros_driver::CustomMsg(*msg_in));

//   if (abs(last_timestamp_imu - msg->header.stamp.toSec()) > 1.0 && !imu_buffer.empty())
//   {
//     double timediff_imu_wrt_lidar = last_timestamp_imu - msg->header.stamp.toSec();
//     printf("\033[95mSelf sync IMU and LiDAR, HARD time lag is %.10lf \n\033[0m", timediff_imu_wrt_lidar - 0.100);
//   }

  double cur_head_time = msg->header.stamp.toSec();
  ROS_INFO("Get LiDAR, its header time: %.6f", cur_head_time);
  if (cur_head_time < last_timestamp_lidar)
  {
    ROS_ERROR("lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);

  if (!ptr || ptr->empty()) {
    ROS_ERROR("Received an empty point cloud");
    mtx_buffer.unlock();
    return;
  }

  // initialize the depth img
  cv::Mat depth(height, width, CV_32FC1, cv::Scalar(0));
  const float z_min = static_cast<float>(depth_min_);
  const float z_max = static_cast<float>(depth_max_);

  for(const auto &pt:ptr->points){
    // LiDAR -> Camera
    Eigen::Vector3d Pl(pt.x, pt.y, pt.z);
    Eigen::Vector3d Pc = R_cl_ * Pl + t_cl_;

    const float X = static_cast<float>(Pc.x());
    const float Y = static_cast<float>(Pc.y());
    const float Z = static_cast<float>(Pc.z());
    if (Z <= z_min || Z >= z_max) continue;  // behind or out of range

    // pinhole projection
    int u = static_cast<int>(fx * X / Z + cx);
    int v = static_cast<int>(fy * Y / Z + cy);
    if (u < 0 || u >= width || v < 0 || v >= height) continue;

    float& d = depth.at<float>(v, u);

    // keep nearest (occlusion handling)
    if (d == 0.0f || Z < d) d = Z;
  }

  cv::Mat depth_u16(height, width, CV_16UC1, cv::Scalar(0));
  for (int y = 0; y < height; ++y) {
    const float* src = depth.ptr<float>(y);
    uint16_t*    dst = depth_u16.ptr<uint16_t>(y);
    for (int x = 0; x < width; ++x) {
      float z = src[x];
      dst[x] = (z > 0.0f) ? static_cast<uint16_t>(std::round(z * 1000.0f)) : 0; // mm
    }
  }

  // lid_raw_data_buffer.push_back(ptr);
  // lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  // mtx_buffer.unlock();
  // sig_buffer.notify_all();
  // Use the same timestamp string for both the left column and filename.
  std::ostringstream ts;
  ts.imbue(std::locale::classic());
  ts.setf(std::ios::fixed);
  ts << std::setprecision(6) << msg->header.stamp.toSec();  // TUM uses 6 decimals
  const std::string tstr = ts.str();

  const std::string filename = tstr + ".png";
  const std::string fullpath = depth_dir_ + "/" + filename;

  if (!cv::imwrite(fullpath, depth_u16)) {
    ROS_ERROR("Failed to write depth image to %s", fullpath.c_str());
    return;
  }

  // Append to rgb.txt in a threadsafe way
  {
    std::lock_guard<std::mutex> lk(io_mutex_dep);
    depth_txt_ << tstr << " depth/" << filename << '\n';
    depth_txt_.flush();
  }
}

void bagtumrgbd::rtk_cbk(const sensor_msgs::NavSatFix::ConstPtr& msg){
  float lat, lon, alt;
  int secs, nsecs;
  lat = msg->latitude;
  lon = msg->longitude;
  alt = msg->altitude;
  double time = msg->header.stamp.toSec();
  


}


void bagtumrgbd::ac1_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg){
  ;
}

