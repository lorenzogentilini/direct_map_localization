#include <direct_map_localization/direct_localization.hpp>

int main(int argc, char **argv){
  ros::init(argc, argv, "direct_localization_node");
  ros::NodeHandle nh("~");

  Localizer localizer_(nh);
  ros::spin();

  return 0;
}