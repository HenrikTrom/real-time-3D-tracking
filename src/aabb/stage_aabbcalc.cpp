#include "aabb/stage_aabbcalc.hpp"

namespace rt3d_tracking{

void AABB_PostProcess(std::vector<data::AABB>& aabbs_in)
    {
        float stepx=0;
        float stepy=0;
        float stepz=0;
        for (auto &aabb : aabbs_in)
        {
            stepx=0;
            stepy=0;
            stepz=0;
            if (aabb.ClassID == 0 || aabb.ClassID == 1) // process hand
            {
                stepx = abs(aabb.Vertices[3](0) - aabb.Vertices[0](0)) * 0.015;
                stepy = abs(aabb.Vertices[1](1) - aabb.Vertices[0](1)) * 0.015;
                stepz = abs(aabb.Vertices[4](2) - aabb.Vertices[0](2)) * 0.015;

                aabb.Vertices[0](0) -= stepx; aabb.Vertices[1](0) -= stepx; aabb.Vertices[4](0) -= stepx; aabb.Vertices[5](0) -= stepx;
                aabb.Vertices[1](1) -= stepy; aabb.Vertices[2](1) -= stepy; aabb.Vertices[5](1) -= stepy; aabb.Vertices[6](1) -= stepy;
                aabb.Vertices[4](2) -= stepz; aabb.Vertices[5](2) -= stepz; aabb.Vertices[6](2) -= stepz; aabb.Vertices[7](2) -= stepz;

                aabb.Vertices[2](0) += stepx; aabb.Vertices[3](0) += stepx; aabb.Vertices[6](0) += stepx; aabb.Vertices[7](0) += stepx;
                aabb.Vertices[0](1) += stepy; aabb.Vertices[3](1) += stepy; aabb.Vertices[4](1) += stepy; aabb.Vertices[7](1) += stepy;
                aabb.Vertices[0](2) += stepz; aabb.Vertices[1](2) += stepz; aabb.Vertices[2](2) += stepz; aabb.Vertices[3](2) += stepz;
            }
            else if (aabb.ClassID == 2){
                stepx = abs(aabb.Vertices[3](0) - aabb.Vertices[0](0)) * 0.015;
                stepy = abs(aabb.Vertices[1](1) - aabb.Vertices[0](1)) * 0.015;
                stepz = abs(aabb.Vertices[4](2) - aabb.Vertices[0](2)) * 0.15;

                aabb.Vertices[0](0) -= stepx; aabb.Vertices[1](0) -= stepx; aabb.Vertices[4](0) -= stepx; aabb.Vertices[5](0) -= stepx;
                aabb.Vertices[1](1) -= stepy; aabb.Vertices[2](1) -= stepy; aabb.Vertices[5](1) -= stepy; aabb.Vertices[6](1) -= stepy;
                aabb.Vertices[4](2) -= stepz; aabb.Vertices[5](2) -= stepz; aabb.Vertices[6](2) -= stepz; aabb.Vertices[7](2) -= stepz;

                aabb.Vertices[2](0) += stepx; aabb.Vertices[3](0) += stepx; aabb.Vertices[6](0) += stepx; aabb.Vertices[7](0) += stepx;
                aabb.Vertices[0](1) += stepy; aabb.Vertices[3](1) += stepy; aabb.Vertices[4](1) += stepy; aabb.Vertices[7](1) += stepy;
                aabb.Vertices[0](2) += stepz; aabb.Vertices[1](2) += stepz; aabb.Vertices[2](2) += stepz; aabb.Vertices[3](2) += stepz;
            }
            else if (aabb.ClassID == 3){
                stepx = -abs(aabb.Vertices[3](0) - aabb.Vertices[0](0)) * 0.005;
                stepy = -abs(aabb.Vertices[1](1) - aabb.Vertices[0](1)) * 0.1;
                stepz = abs(aabb.Vertices[4](2) - aabb.Vertices[0](2)) * 0.01;

                aabb.Vertices[0](0) -= stepx; aabb.Vertices[1](0) -= stepx; aabb.Vertices[4](0) -= stepx; aabb.Vertices[5](0) -= stepx;
                aabb.Vertices[1](1) -= stepy; aabb.Vertices[2](1) -= stepy; aabb.Vertices[5](1) -= stepy; aabb.Vertices[6](1) -= stepy;
                aabb.Vertices[4](2) -= stepz; aabb.Vertices[5](2) -= stepz; aabb.Vertices[6](2) -= stepz; aabb.Vertices[7](2) -= stepz;
                
                aabb.Vertices[2](0) += stepx; aabb.Vertices[3](0) += stepx; aabb.Vertices[6](0) += stepx; aabb.Vertices[7](0) += stepx;
                aabb.Vertices[0](1) += stepy; aabb.Vertices[3](1) += stepy; aabb.Vertices[4](1) += stepy; aabb.Vertices[7](1) += stepy;
                aabb.Vertices[0](2) += stepz; aabb.Vertices[1](2) += stepz; aabb.Vertices[2](2) += stepz; aabb.Vertices[3](2) += stepz;
            }
        }
    }

namespace stages{

AABBCalculate::AABBCalculate(
    const config_aabbcalc &cfg_aabbcalc, 
    const flir_icp_calib::MultiCameras &cameras) : cfg_aabbcalc(cfg_aabbcalc), cameras(cameras)
{
    this->postProcess = AABB_PostProcess;
    this->ThreadHandle.reset(new std::thread(&AABBCalculate::ThreadFunction, this));
}

bool AABBCalculate::ProcessFunction(
    std::vector<std::vector<data::BoundingBox2D>> &aabbcalc_in,
    std::vector<data::AABB> &aabbcalc_out
){
    // auto &BB2Ds = this->InFIFO.front();

    std::vector<std::future<data::AABB>> AABBs_temp;
    for (auto &BB2D : aabbcalc_in)
    {
        AABBs_temp.push_back(async(std::launch::async, CalcAABBfrom2DBBs, BB2D, this->cfg_aabbcalc.UseConvexhull, this->cameras));
    }
    std::this_thread::sleep_for(std::chrono::microseconds(1)); // is this needed for synchronisation
    for (auto &aabb_tmp : AABBs_temp)
    {
        auto aabb = aabb_tmp.get();
        aabbcalc_out.push_back(aabb);
    }
    this->postProcess(aabbcalc_out);
    
    return true;
}

void AABBCalculate::Terminate(void){
    this->ShouldClose=true;
    this->ThreadHandle->join();
}

} // namespace stages

} //namespace rt3d_tracking