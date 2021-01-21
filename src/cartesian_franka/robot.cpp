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
        JointMotionGenerator joint_motion_generator(0.5, q_goal);
        _robot.control(joint_motion_generator);
    }

} // namespace cartesian_franka