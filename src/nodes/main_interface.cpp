#include "tracker/tracker.hpp"
#include "ros-node-interface/interface.hpp"

using namespace rt3d_tracking;

int main(int argc, char **argv) {
    ros::init(argc, argv, "tracking_interface");
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