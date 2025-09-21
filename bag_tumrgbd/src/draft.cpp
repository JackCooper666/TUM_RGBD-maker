#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/filesystem.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <unistd.h>   // readlink
#include <limits.h>   // PATH_MAX
#include <cstdlib>
#include <iomanip>
#include <sstream>

namespace fs  = boost::filesystem;
namespace enc = sensor_msgs::image_encodings;

class bagtumrgbd {
public:
  explicit bagtumrgbd(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : it_(nh) {                       // IMPORTANT: initialize ImageTransport
    initializeParams(pnh);
    initializeFolder(pnh);
    initializeSubscriber(nh);
  }

  ~bagtumrgbd() = default;

private:
  // ---- params / setup -------------------------------------------------------
  void initializeParams(ros::NodeHandle& pnh) {
    // param name: common/img_topic (private or global both OK if fully qualified here)
    pnh.param<std::string>("common/img_topic", img_topic_, std::string("/rs_camera/rgb"));
  }

  static std::string guessPackageNameFromExe() {
    char buf[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (len <= 0) return {};
    buf[len] = '\0';
    fs::path exe(buf);                             // .../lib/<pkg>/<node>
    return exe.parent_path().filename().string();  // <pkg>
  }

  static std::string getPackagePathAuto() {
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

  void initializeFolder(ros::NodeHandle& pnh) {
    pnh.param<std::string>("output_dir", output_dir_param_, std::string(""));

    if (!output_dir_param_.empty()) {
      out_dir_ = output_dir_param_;
    } else {
      const std::string pkg_path = getPackagePathAuto();
      if (!pkg_path.empty()) {
        out_dir_ = pkg_path + "/src/output/imgs";
      } else {
        ROS_WARN("Could not resolve package path automatically; falling back to $HOME/ros_imgs");
        const char* home = std::getenv("HOME");
        out_dir_ = (home ? std::string(home) : std::string("/tmp")) + "/ros_imgs";
      }
    }

    try {
      if (!fs::exists(out_dir_)) {
        fs::create_directories(out_dir_);
      }
    } catch (const fs::filesystem_error& e) {
      ROS_FATAL("Failed to create output directory '%s': %s", out_dir_.c_str(), e.what());
      throw;
    }

    ROS_INFO("Images will be saved to: %s", out_dir_.c_str());
  }

  void initializeSubscriber(ros::NodeHandle& nh) {
    // Reasonable queue size; adjust if your bag is very bursty
    sub_img_ = it_.subscribe(img_topic_, 10, &bagtumrgbd::imgCbk, this);
    ROS_INFO("Subscribing to image topic: %s", img_topic_.c_str());
  }

  // ---- callback -------------------------------------------------------------
  void imgCbk(const sensor_msgs::ImageConstPtr& msg) {
    try {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
      cv::Mat to_save;

      if (msg->encoding == enc::RGB8) {
        cv::cvtColor(cv_ptr->image, to_save, cv::COLOR_RGB2BGR);
      } else if (msg->encoding == enc::BGR8 || msg->encoding == enc::MONO8 || msg->encoding == enc::MONO16) {
        to_save = cv_ptr->image;
      } else {
        // Convert other color encodings to BGR8 for portability
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
        to_save = cv_ptr->image;
      }

      const uint64_t ns = msg->header.stamp.toNSec();
      std::ostringstream oss;
      oss << out_dir_ << "/img_" << std::setw(19) << std::setfill('0') << ns << ".png";

      if (!cv::imwrite(oss.str(), to_save)) {
        ROS_ERROR("Failed to write image to %s", oss.str().c_str());
      } else {
        if ((++saved_cnt_ % 50) == 0) {
          ROS_INFO("Saved %zu images (latest: %s)", saved_cnt_, oss.str().c_str());
        }
      }
    } catch (const cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }

private:
  image_transport::ImageTransport it_;
  image_transport::Subscriber     sub_img_;

  std::string img_topic_;
  std::string output_dir_param_;
  std::string out_dir_;
  std::size_t saved_cnt_{0};
};

// ---- main -------------------------------------------------------------------
int main(int argc, char** argv) {
  ros::init(argc, argv, "rgb_saver");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    bagtumrgbd node(nh, pnh);
    ros::spin();
  } catch (...) {
    ROS_ERROR("rgb_saver terminated due to initialization error.");
    return 1;
  }
  return 0;
}
