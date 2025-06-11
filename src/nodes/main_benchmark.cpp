#include "tracker/tracker.hpp"

using namespace rt3d_tracking;

// open images
// new stage
// detect images -> write bbs
int main(int argc, char **argv){
    ros::init(argc, argv, "online_tracking");
    ros::NodeHandle nh("~");
    std::unique_ptr<modules::TrackingInterfaceModule> trackingInterfaceModule;
    if (!modules::init_trackingInterfaceModule(nh, trackingInterfaceModule)){
        return 1;
    };
    ros_node_interface::BaseRosInterface<modules::TrackingInterfaceModule> tracking_interface(
        std::move(trackingInterfaceModule)
    );
    spdlog::info("Main thread exiting (tracking_interface)...");

    return 0;
}