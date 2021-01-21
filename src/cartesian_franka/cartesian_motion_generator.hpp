#ifndef _FRANKA_CARTESIAN_GENERATOR_HPP_
#define _FRANKA_CARTESIAN_GENERATOR_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>

#include <franka/exception.h>
#include <franka/robot.h>

namespace cartesian_franka {
    class CartesianMotionGenerator {
    public:
        ///movements are relative !
        CartesianMotionGenerator(const Eigen::Affine3d& target, double duration = -1)
            : _target(target), _duration(duration), _time(0) {
            }

        franka::CartesianPose operator()(const franka::RobotState& robot_state, franka::Duration period);

    protected:
        void _init(const franka::RobotState& robot_state);
        double _traj(double time, double duration) const;

        // initial values (will be initialized in the operator() at t=0)
        Eigen::Affine3d _i_transform;
        Eigen::Vector3d _i_translation;
        Eigen::Quaterniond _i_rotation;
        // target
        Eigen::Affine3d _target;
        Eigen::Quaterniond _target_rotation; // computed from _target

        double _duration;
        double _time;
    };
} // namespace cartesian_franka
#endif