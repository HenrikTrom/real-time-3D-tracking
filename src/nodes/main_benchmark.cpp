#include "tracker/tracker.hpp"

using namespace rt3d_tracking;

int main(int argc, char **argv){
    ros::init(argc, argv, "online_tracking");
    ros::NodeHandle nh("~");
    std::unique_ptr<modules::TrackingInterfaceModule> trackingInterfaceModule;
    if (!modules::init_trackingInterfaceModule(nh, trackingInterfaceModule)){
        return 1;
    };
    trackingInterfaceModule->start();

    while (!trackingInterfaceModule->ShouldClose){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    spdlog::info("Main thread exiting (tracking_interface)...");
    trackingInterfaceModule->Terminate();

    return 0;
}