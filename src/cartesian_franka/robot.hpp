#ifndef _FRANKA_ROBOT_HPP_
#define _FRANKA_ROBOT_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>

#include <franka/exception.h>
#include <franka/robot.h>

namespace cartesian_franka { // cf = cartesian_franka
    class Robot {
    public:

        /// take the IP of the robot as input
        Robot(const std::string& ip) : _robot(ip.c_str())
        {
            _set_default_behavior();
            init();
        }

        /// go to the starting position using joint positions
        void init(double duration = 0.5);

        // move to a position + rotation
        void move_cartesian_relative(const Eigen::Affine3d& end_position, double duration = -1);
        void move_cartesian_relative(const Eigen::Vector3d& end_position, const Eigen::Vector3d& rpy, double duration = -1);

        // move to a joint position
        void move_joint_absolute(const std::array<double, 7>& joint_positions, double duration);

        /// end-effector transform (position & rotation)
        Eigen::Affine3d position();

        /// the libfranka robot if needed for other functions
        const franka::Robot& franka() const { return _robot; }

    protected:
        void _set_default_behavior();
        franka::Robot _robot;
    };
} // namespace cartesian_franka

#endif