#include "main.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "fast_lio_sam_node");
  ros::NodeHandle nh_private("~");

  FAST_LIO_SAM_CLASS fast_lio_sam_(nh_private);

  ros::AsyncSpinner spinner(4); // Use multi threads
  spinner.start();
  ros::waitForShutdown();

  fast_lio_sam_.~FAST_LIO_SAM_CLASS();

  return 0;
}