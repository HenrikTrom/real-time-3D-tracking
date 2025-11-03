#include "config_parser.hpp"

namespace rt3d_tracking{

bool load_tracking_config(config_tracking &cfg_pipeline){
    const std::string docpath = std::string(CONFIG_DIR)+"/SettingsTracking.json";
    const std::string schemepath = std::string(CONFIG_DIR)+"/SettingsTracking.schema.json"; // compare with scheme
    rapidjson::Document doc;
    if(!cpp_utils::load_json_with_schema(docpath, schemepath, 65536, doc)){
        throw std::runtime_error("Could not load pipeline config");
        return false;
    }
    cfg_pipeline.cfg_multicamera = doc["cfg_multicamera"].GetString();
    cfg_pipeline.cfg_camera_calibration = doc["cfg_camera_calibration"].GetString();
    cfg_pipeline.cfg_det = doc["cfg_det"].GetString();
    cfg_pipeline.cfg_pose = doc["cfg_pose"].GetString();
    cfg_pipeline.online_mode = doc["online_mode"].GetBool();
    cfg_pipeline.maf_window_size = doc["maf_window_size"].GetUint64();
    cfg_pipeline.filter_conf_threshold = doc["filter_conf_threshold"].GetDouble();
    return true;
}

}