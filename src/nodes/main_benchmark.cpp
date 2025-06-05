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
    trackingInterfaceModule->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    trackingInterfaceModule->Terminate();

    return 0;
}