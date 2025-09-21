#include "bag_tumrgbd/bagsub.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "bag_tumrgbd");
  ros::NodeHandle nh;

  bagtumrgbd node(nh);
  ros::spin();
  return 0;
}
