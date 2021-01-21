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

        // all these function are relative to the current position/orientation
        void translate(const Eigen::Vector3d& delta, double duration = -1);
        void rotate(const Eigen::Vector3d& rpy, double duration = -1);
        void move(const Eigen::Vector3d& delta, const Eigen::Vector3d& rpy, double duration = -1);
        void move(const Eigen::Affine3d& transform_delta, double duration = -1);

        // move to a joint position
        void move_joints(const std::array<double, 7>& joint_positions, double duration = 1);

        /// end-effector transform (position & rotation)
        Eigen::Affine3d affine3d();
        Eigen::Vector3d position();
        Eigen::Vector3d orientation();

        /// the libfranka robot if needed for other functions
        const franka::Robot& franka() const { return _robot; }

    protected:
        void _set_default_behavior();
        franka::Robot _robot;
    };
} // namespace cartesian_franka

#endif