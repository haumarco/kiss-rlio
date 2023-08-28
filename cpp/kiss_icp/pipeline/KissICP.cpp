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
    // std::cout << "sigma: " << sigma << std::endl;

    // Compute initial_guess for ICP
    const auto prediction = GetPredictionModel();
    Eigen::Quaterniond initial_rotation(0.0, 0.7071068, -0.7071068, 0.0 ); // 90 deg z rot + 180 deg x rot
    // const auto last_pose_old = !poses_.empty() ? poses_.back() : Sophus::SE3d(initial_rotation, Eigen::Vector3d(0,0,0));
    const auto last_pose_old = !poses_.empty() ? poses_.back() : Sophus::SE3d();
    const auto initial_guess_old = last_pose_old * prediction;

    const auto initial_guess = get_initial_guess_gtsam();

    // std::cout << "norm initial_guess: " << gtsam_odom_translation.normalized() << std::endl;

    // bool degenerate = false;

    // Run icp
    Sophus::SE3d new_pose = kiss_icp::RegisterFrame(source,         //
                                                          local_map_,     //
                                                          initial_guess,  ////////////////////////////////////
                                                          3.0 * sigma,    //
                                                          sigma / 3.0,
                                                          degenerate,
                                                          gtsam_odom_translation.normalized());

    // std::cout << "degenerate: " << degenerate << std::endl;
    // std::cout << "sigma: " << sigma << std::endl;

    const auto model_deviation = initial_guess.inverse() * new_pose;
    const auto model_deviation_old = initial_guess_old.inverse() * new_pose;
   
    // const auto predicted_diff = last_pose_old.inverse() * initial_guess; ////////
    // const auto estimated_diff = last_pose_old.inverse() * new_pose;

    // double predicted_trans_diff = std::sqrt(predicted_diff.translation().x()*predicted_diff.translation().x() + predicted_diff.translation().y()*predicted_diff.translation().y() + predicted_diff.translation().z()*predicted_diff.translation().z());
    // double estimated_trans_diff = std::sqrt(estimated_diff.translation().x()*estimated_diff.translation().x() + estimated_diff.translation().y()*estimated_diff.translation().y() + estimated_diff.translation().z()*estimated_diff.translation().z());
    
    // Eigen::AngleAxisd angleAxis_predicted(predicted_diff.rotationMatrix());
    // Eigen::Vector3d rotationVector_predicted = angleAxis_predicted.angle() * angleAxis_predicted.axis();
    // double predicted_rot_diff = rotationVector_predicted.norm();
    // Eigen::AngleAxisd angleAxis_estimated(estimated_diff.rotationMatrix());
    // Eigen::Vector3d rotationVector_estimated = angleAxis_estimated.angle() * angleAxis_estimated.axis();
    // double estimated_rot_diff = rotationVector_estimated.norm();

    // double trans_diff_ratio = estimated_trans_diff / predicted_trans_diff;
    // double rot_diff_ratio = estimated_rot_diff / predicted_rot_diff;
    // std::cout << "trans_diff_ratio: " << trans_diff_ratio << std::endl;
    // std::cout << "rot_diff_ratio: " << rot_diff_ratio << std::endl;

    // if(trans_diff_ratio < 0.33 || trans_diff_ratio > 3){
    // // if(trans_diff_ratio > 3){
    //     std::cout << "dont trust lidar translation: "<< trans_diff_ratio << " : " << predicted_trans_diff  << std::endl;
    //     // new_pose = Sophus::SE3d(initial_guess);
    //     // new_pose = initial_guess;
    //     degenerate = true;
    // }

    // if(rot_diff_ratio < 0.33 || rot_diff_ratio > 3){
    // // if(rot_diff_ratio > 3){
    //     std::cout << "dont trust lidar rotation" << rot_diff_ratio << " : " << predicted_rot_diff << std::endl;
    //     // new_pose = Sophus::SE3d(initial_guess);
    //     // new_pose = initial_guess;
    //     degenerate = true;
    // }


    // std::cout << "model deviation old;" << model_deviation_old.translation().x() << ";" << model_deviation_old.translation().y() << ";" << model_deviation_old.translation().z() << std::endl;
    // std::cout << "model deviation;" << model_deviation.translation().x() << ";" << model_deviation.translation().y() << ";" << model_deviation.translation().z() << std::endl;
    


    // double model_dev_rms = std::sqrt(model_deviation.translation().x()*model_deviation.translation().x() + model_deviation.translation().y()*model_deviation.translation().y() + model_deviation.translation().z()*model_deviation.translation().z());
    // Eigen::AngleAxisd angleAxis(model_deviation.rotationMatrix());
    // Eigen::Vector3d rotationVector = angleAxis.angle() * angleAxis.axis();
    // // Compute the magnitude of the rotation vector to get the error
    // double error = rotationVector.norm();
    // std::cout << "trans model dev: " << model_dev_rms << std::endl;
    // std::cout << "rot model dev: " << error << std::endl;

    // std::cout << "initial_guess: " << predicted_diff.translation().x() << ";" << predicted_diff.translation().y() << ";" << predicted_diff.translation().z() << std::endl;

    // ATTENTION
    if (poses_.size() % 100000 == 0) {
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


// with this approach I givbe the gtsam position as a prior
// Sophus::SE3d KissICP::get_initial_guess_gtsam() {

//     double dt = gtsam_current_msg_time - gtsam_last_msg_time;
//     gtsam_last_msg_time = gtsam_current_msg_time;

//     Sophus::SE3d transformation_change(Sophus::SO3d::exp(gtsam_odom_rotation*dt), gtsam_odom_translation*dt);

//     return gtsam_initial_pose * transformation_change;
// }

 
Sophus::SE3d KissICP::get_initial_guess_gtsam() {

    Sophus::SE3d::Point translation_point(gtsam_odom_translation);

    Sophus::SE3d se3_transform(gtsam_odom_rotation.toRotationMatrix(), translation_point);

    auto last_pose = !poses_.empty() ? poses_.back() : Sophus::SE3d();

    auto predicted_pose = last_pose * se3_transform;

    return predicted_pose;


}



}  // namespace kiss_icp::pipeline