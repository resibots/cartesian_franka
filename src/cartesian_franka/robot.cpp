#include <Eigen/Core>
#include <Eigen/Dense>

#include "cartesian_motion_generator.hpp"
#include "joint_motion_generator.hpp"
#include "robot.hpp"

namespace cartesian_franka {
    void Robot::_set_default_behavior()
    {
        _robot.setCollisionBehavior(
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
        _robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
        _robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
    }

    void Robot::init(double duration)
    {
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, 0}};
        move_joints(q_goal, 0.5);
    }

    void Robot::move(const Eigen::Affine3d& end_position, double duration)
    {
        CartesianMotionGenerator generator(end_position, duration);
        _robot.control(generator);
    }

    void Robot::move(const Eigen::Vector3d& end_position, const Eigen::Vector3d& rpy, double duration)
    {
        Eigen::Affine3d t = Eigen::Translation3d(end_position)
            * Eigen::AngleAxisd(rpy[0] * M_PI, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(rpy[1] * M_PI, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(rpy[2] * M_PI, Eigen::Vector3d::UnitZ());
        assert(t.translation() == end_position);
        move(t, duration);
    }

    void Robot::rotate(const Eigen::Vector3d& rpy, double duration)
    {
        Eigen::Quaterniond q = Eigen::AngleAxisd(rpy[0] * M_PI, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(rpy[1] * M_PI, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(rpy[2] * M_PI, Eigen::Vector3d::UnitZ());
        Eigen::Affine3d t(q);
        move(t, duration);

    }

     void Robot::translate(const Eigen::Vector3d& delta, double duration)
    {    
        Eigen::Affine3d t = Eigen::Affine3d::Identity();
        t.translation() = delta;
        move(t, duration);
    }

    void Robot::move_joints(const std::array<double, 7>& joint_positions, double duration)
    {
        JointMotionGenerator joint_motion_generator(duration, joint_positions);
        _robot.control(joint_motion_generator);
    }

    Eigen::Vector3d Robot::position()
    {
        return affine3d().translation();
    }

    Eigen::Vector3d Robot::orientation()
    {
        Eigen::Quaterniond q(affine3d().linear());
        return q.toRotationMatrix().eulerAngles(0, 1, 2);
    }

    Eigen::Affine3d Robot::affine3d()
    {
        franka::RobotState robot_state = _robot.readOnce();
        Eigen::Matrix4d m; // = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
        // the map should work...
        for (int i = 0; i < 16; ++i)
            m.data()[i] = robot_state.O_T_EE_c[i];
        Eigen::Affine3d transform(m);
        assert(transform.translation()[0] == robot_state.O_T_EE_c[12]);
        assert(transform.translation()[1] == robot_state.O_T_EE_c[13]);
        assert(transform.translation()[2] == robot_state.O_T_EE_c[14]);
        return transform;
    }

} // namespace cartesian_franka