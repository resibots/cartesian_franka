#include <iostream>

#include "cartesian_motion_generator.hpp"

namespace cartesian_franka {

    // this is the profile between 0 and 1, for a specific duration
    // (this looks like a sigmoid)
    double CartesianMotionGenerator::_traj(double time, double duration) const
    {
        return sin(M_PI / 4 * (1 - std::cos(M_PI / duration * time)));
    }

    franka::CartesianPose CartesianMotionGenerator::operator()(const franka::RobotState& robot_state, franka::Duration period)
    {
        _time += period.toSec();

        if (_time == 0.0) {
            Eigen::Matrix4d m; // = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
            // the map should work...
            for (int i = 0; i < 16; ++i)
                m.data()[i] = robot_state.O_T_EE_c[i];
            _i_transform = m;
            _i_translation = _i_transform.translation();
            _i_rotation = Eigen::Quaterniond(_i_transform.linear());
            _target_rotation = Eigen::Quaterniond(_target.linear()) * _i_rotation; //relative
            assert(_i_transform.translation()[0] == robot_state.O_T_EE_c[12]);
            assert(_i_transform.translation()[1] == robot_state.O_T_EE_c[13]);
            assert(_i_transform.translation()[2] == robot_state.O_T_EE_c[14]);
            if (_duration == -1)
                _duration = (_i_translation - _target.translation()).norm() / 0.10; // 10 cm/s by default
            std::cout<<"duration:"<<_duration<<std::endl;
        }

        // interpolate
        double x = _traj(_time, _duration);
        Eigen::Vector3d delta = x * _target.translation(); // delta is relative motion
        Eigen::Quaterniond q = _i_rotation.slerp(x, _target_rotation);

        // put everything in a homogogenous matrix
        Eigen::Affine3d transform;
        transform.linear() = q.normalized().toRotationMatrix();
        transform.translation() = _i_translation + delta;

        // copy to the array
        std::array<double, 16> new_pose;
        std::copy_n(transform.data(), new_pose.size(), new_pose.begin());

        if (_time >= _duration) {
            return franka::MotionFinished(new_pose);
        }
        return new_pose;
    }

} // namespace cartesian_franka