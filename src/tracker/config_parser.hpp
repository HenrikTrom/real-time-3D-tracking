#pragma once
#include "cpp_utils/jsontools.h"
#include "../config.h"

namespace rt3d_tracking {

struct config_tracking
{
    std::string cfg_multicamera = "ERRORcfg_multicameraNotSpecified";
    std::string cfg_camera_calibration = "ERRORcfg_camera_calibrationNotSpecified";
    std::string cfg_det = "ERRORcfg_detNotSpecified";
    std::string cfg_pose = "ERRORcfg_bposeNotSpecified";
    int min_cams = 0;
    bool online_mode = false; // live images y/n?
    int compression_quality = 100;
    std::vector<int> compression_params;
};

// TODO: Understand correspondance
// TODO: parse, remove solid/non-solid
// TODO: fix threshold error
struct config_correspondance
{
    uint8_t NumOfClasses = 4;
    uint8_t HandID = 1; // if Hand->NonSolidObjectDistanceThreshold
    float SolidObjectDistanceThreshold = 60.0; //30
    float NonSolidObjectDistanceThreshold = 100.0; //50
    uint8_t ObjectAtLeastSeenBy = 2;
};

// TODO: parse
struct config_aabbcalc
{
    uint8_t NumOfClasses = 4;
    bool UseConvexhull = false;
};

bool load_tracking_config(config_tracking &cfg_pipeline);

}