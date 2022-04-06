#include "ray_filter_ground/ray_filter_core.h"
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ray_filter_ground");
    
    ros::NodeHandle nh;

    PclCore core(nh);

    return 0;
}
