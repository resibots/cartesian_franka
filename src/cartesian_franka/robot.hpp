#ifndef _FRANKA_ROBOT_HPP_
#define _FRANKA_ROBOT_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>

#include <franka/exception.h>
#include <franka/robot.h>

namespace cartesian_franka { // cf = cartesian_franka
    class Robot {
    public:
        using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;

        /// take the IP of the robot as input
        Robot(const std::string& ip) : _robot(ip.c_str())
        {
            _set_default_behavior();
        }

        /// go to the starting position using joint positions
        void init(double duration = 0.5);

        // move to a position + rotation
        void move_cartesian_relative(const Eigen::Affine3d& end_position);
        void move_cartesian_relative(const Eigen::Vector3d& end_position, const Eigen::Vector3d& rpy)
        {
            Eigen::Affine3d t = Eigen::Translation3d(end_position)
                * Eigen::AngleAxisd(rpy[0] * M_PI, Eigen::Vector3d::UnitX())
                * Eigen::AngleAxisd(rpy[1] * M_PI, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(rpy[2] * M_PI, Eigen::Vector3d::UnitZ());
            assert(t.translation() == end_position);
            move_cartesian_relative(t);
        }

        // move to a joint position
        void move_joint_absolute(const Vector7d& joint_positions);

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