#include "fast_lio_sam.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fast_lio_sam_node");
    ros::NodeHandle nh_private("~");

    FastLioSam fast_lio_sam_(nh_private);

    ros::AsyncSpinner spinner(4); // Use multi threads
    spinner.start();
    ros::waitForShutdown();

    fast_lio_sam_.~FastLioSam(); // Explicit call of destructor

    return 0;
}
