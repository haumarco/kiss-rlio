// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "KissICP.hpp"

#include <Eigen/Core>
#include <tuple>
#include <vector>

#include "kiss_icp/core/Deskew.hpp"
#include "kiss_icp/core/Preprocessing.hpp"
#include "kiss_icp/core/Registration.hpp"
#include "kiss_icp/core/VoxelHashMap.hpp"

#include <iostream>

namespace kiss_icp::pipeline {

KissICP::Vector3dVectorTuple KissICP::RegisterFrame(const std::vector<Eigen::Vector3d> &frame,
                                                    const std::vector<double> &timestamps) {
    return RegisterFrame(frame); // because I currently do deskewing in odometryserver
    const auto &deskew_frame = [&]() -> std::vector<Eigen::Vector3d> {
        if (!config_.deskew) return frame;
        // TODO(Nacho) Add some asserts here to sanitize the timestamps

        //  If not enough poses for the estimation, do not de-skew
        const size_t N = poses().size();
        if (N <= 2) return frame;

        // Estimate linear and angular velocities
        const auto &start_pose = poses_[N - 2];
        const auto &finish_pose = poses_[N - 1];
        return DeSkewScan(frame, timestamps, start_pose, finish_pose);
    }();
    return RegisterFrame(deskew_frame);
}

KissICP::Vector3dVectorTuple KissICP::RegisterFrame(const std::vector<Eigen::Vector3d> &frame) {
    // Preprocess the input cloud
    const auto &cropped_frame = Preprocess(frame, config_.max_range, config_.min_range);

    // Voxelize
    const auto &[source, frame_downsample] = Voxelize(cropped_frame);

    // Get motion prediction and adaptive_threshold
    const double sigma = GetAdaptiveThreshold();

    // Compute initial_guess for ICP
    const auto prediction = GetPredictionModel();
    Eigen::Quaterniond initial_rotation(0.0, 0.7071068, -0.7071068, 0.0 ); // 90 deg z rot + 180 deg x rot
    // const auto last_pose_old = !poses_.empty() ? poses_.back() : Sophus::SE3d(initial_rotation, Eigen::Vector3d(0,0,0));
    const auto last_pose_old = !poses_.empty() ? poses_.back() : Sophus::SE3d();
    const auto initial_guess_old = last_pose_old * prediction;

    const auto initial_guess = get_initial_guess_gtsam();


    // Run icp
    Sophus::SE3d new_pose = kiss_icp::RegisterFrame(source,         //
                                                          local_map_,     //
                                                          initial_guess,  //
                                                          3.0 * sigma,    //
                                                          sigma / 3.0,
                                                          degenerate,
                                                          gtsam_odom_translation.normalized());

    // std::cout << "degenerate: " << degenerate << std::endl;
    // std::cout << "sigma: " << sigma << std::endl;

    const auto model_deviation = initial_guess.inverse() * new_pose;
    const auto model_deviation_old = initial_guess_old.inverse() * new_pose;
   

    if (poses_.size() % 100000 == 0) {
    // if (poses_.size() % 100 == 0) {
        local_map_.Clear();
    }
    adaptive_threshold_.UpdateModelDeviation(model_deviation);
    local_map_.Update(frame_downsample, new_pose);
    poses_.push_back(new_pose);
    return {frame, source};
}

KissICP::Vector3dVectorTuple KissICP::Voxelize(const std::vector<Eigen::Vector3d> &frame) const {
    const auto voxel_size = config_.voxel_size;
    const auto frame_downsample = kiss_icp::VoxelDownsample(frame, voxel_size * 0.5);
    const auto source = kiss_icp::VoxelDownsample(frame_downsample, voxel_size * 1.5);
    return {source, frame_downsample};
}

double KissICP::GetAdaptiveThreshold() {
    if (!HasMoved()) {
        return config_.initial_threshold;
    }
    return adaptive_threshold_.ComputeThreshold();
}

Sophus::SE3d KissICP::GetPredictionModel() const {
    Sophus::SE3d pred = Sophus::SE3d();
    const size_t N = poses_.size();
    if (N < 2) return pred;
    return poses_[N - 2].inverse() * poses_[N - 1];
}

bool KissICP::HasMoved() {
    if (poses_.empty()) return false;
    const double motion = (poses_.front().inverse() * poses_.back()).translation().norm();
    return motion > 5.0 * config_.min_motion_th;
}



 
Sophus::SE3d KissICP::get_initial_guess_gtsam() {

    Sophus::SE3d::Point translation_point(gtsam_odom_translation);

    Sophus::SE3d se3_transform(gtsam_odom_rotation.toRotationMatrix(), translation_point);

    auto last_pose = !poses_.empty() ? poses_.back() : Sophus::SE3d();

    auto predicted_pose = last_pose * se3_transform;

    return predicted_pose;


}



}  // namespace kiss_icp::pipeline