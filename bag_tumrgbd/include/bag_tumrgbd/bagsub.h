#include <string>
#include <cstddef>
#include <fstream>
#include <mutex>

#include <ros/ros.h>
#include <ros/package.h>

// img
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
// imu
#include <sensor_msgs/Imu.h>
// mocap
#include <geometry_msgs/PoseStamped.h> 
// file saving
#include <boost/filesystem.hpp>
// rtk
#include <sensor_msgs/NavSatFix.h>

#include <condition_variable>

#include <unistd.h>   // readlink
#include <limits.h>   // PATH_MAX
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <locale>

#include <bag_tumrgbd/preprocess.h>

// #define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
// #define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]


class bagtumrgbd
{
public:
    bagtumrgbd(ros::NodeHandle& nh);
    ~bagtumrgbd();

    // subscriber switcher
    bool lidar_en = true;

    // ground switch
    bool use_rtk;

    // img subscriber
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_img;

    // imu subscriber
    ros::Subscriber sub_imu;

    // mocap subscriber
    ros::Subscriber sub_mocap;
   
    // pcl subscriber
    ros::Subscriber sub_pcl;

    // rtk subscriber
    ros::Subscriber sub_rtk;

    // rtk cb

    // std::string lid_topic, imu_topic, img_topic;

    // subscribed topics
    std::string img_topic_;
    std::string imu_topic_;
    std::string mocap_topic_;
    std::string rtk_topic_;
    std::string lid_topic_;


    // file path base
    // std::string output_dir_param;
    std::string out_dir;

    // imgs path
    std::string rgb_dir_;
    std::string rgb_txt_path_;
    std::ofstream rgb_txt_; // imgs' detail file
    std::mutex io_mutex_img;

    // imu txt path
    std::string imu_txt_path_;
    std::ofstream imu_txt_; // imu's detail file
    std::mutex io_mutex_imu;

    // mocap txt path
    std::string mocap_txt_path_;
    std::ofstream mocap_txt_;
    std::mutex io_mutex_mocap;
    static constexpr int kGT_PREC = 6;

    // rtk txt path
    std::string rtk_txt_path_;
    std::ofstream rtk_txt_;
    std::mutex io_mutex_rtk;

    // depth img path 
    std::string depth_dir_;
    std::string depth_txt_path_;
    std::ofstream depth_txt_;
    std::mutex io_mutex_dep;


    // std::size_t saved_cnt_{0};

    // pcl preprocess
    PreprocessPtr p_pre;

    // camera intrinsic 
    double fx;
    double fy;
    double cx;
    double cy;
    int height;
    int width;

    // Depth settings
    double depth_min_;     // meters
    double depth_max_;    // meters

    // extrinsic
    // vector<double> extrinT;
    // vector<double> extrinR;
    vector<double> cameraextrinT;
    vector<double> cameraextrinR;

    // Lidar->Camera extrinsics (Camera <- LiDAR)
    Eigen::Matrix3d R_cl_;      // rotation
    Eigen::Vector3d t_cl_;      // translation


    // for sync
    std::mutex mtx_buffer, mtx_buffer_imu_prop;
    std::condition_variable sig_buffer;
    double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0;
    
    double img_time_offset = 0.0;
    deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;
    deque<double> lid_header_time_buffer;
    deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
    deque<cv::Mat> img_buffer;
    deque<double> img_time_buffer;
    

    // build and find the output dir
    std::string guessPackageNameFromExe();
    std::string getPackagePathAuto();

    // read yaml
    void initializeparams(ros::NodeHandle& nh);
    
    // build img dir and depth img dir
    void initializefolder();
    
    // img log
    void set_img_dir_img_txt();
    // accelerator log
    void set_imu_txt();
    // ground truth log
    void set_mocap_txt(); // mocap version
    void set_dep_dir_dep_txt();

    // initialize subscriber
    void initializesubscriber(ros::NodeHandle& nh);

    // read imu data
    void imu_cbk(const sensor_msgs::ImuConstPtr& msg);
    // read img data
    void img_cbk(const sensor_msgs::ImageConstPtr& msg);

    // read ground truth
    // for rtk
    void rtk_cbk(const sensor_msgs::NavSatFix::ConstPtr& msg);
    // for mocap
    void mocap_cbk(const geometry_msgs::PoseStampedConstPtr& msg);
    
    // read pcl and convert to depth img
    void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in);
    void ac1_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
    
    
    void run();
};
