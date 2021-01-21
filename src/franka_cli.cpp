// Copyright (c) 2021 Inria

#include <cassert>
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
#include <Eigen/Core>
#include <Eigen/Dense>



int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  try {
//     franka::Robot robot(argv[1]);
//     setDefaultBehavior(robot);

//     // First move the robot to a suitable joint configuration
//     std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, 0}};
//     JointMotionGenerator joint_motion_generator(0.5, q_goal);
//     std::cout << "WARNING: This example will move the robot! "
//               << "Please make sure to have the user stop button at hand!" << std::endl
//               << "Press Enter to continue..." << std::endl;
//     std::cin.ignore();
//     robot.control(joint_motion_generator);
//     std::cout << "Finished moving to initial joint configuration." << std::endl;

//     // Set additional parameters always before the control loop, NEVER in the control loop!
//     // Set collision behavior.
   

//     std::array<double, 16> initial_pose;
//     double time = 0.0;
//     Eigen::Vector3d target(0.1, 0.0, 0.0);  // 10 cm / s


//     Eigen::Quaterniond target_rotation =
//         Eigen::AngleAxisd(0.25 * M_PI, Eigen::Vector3d::UnitX())
//       * Eigen::AngleAxisd(0.00 * M_PI, Eigen::Vector3d::UnitY())
//       * Eigen::AngleAxisd(0.00 * M_PI, Eigen::Vector3d::UnitZ());
//     double target_time = 3.0;

//     Eigen::Affine3d initial_transform;
//     Eigen::Vector3d initial_pos;
//     Eigen::Quaterniond initial_orientation;

//     robot.control([&](const franka::RobotState& robot_state,
//                       franka::Duration period) -> franka::CartesianPose {
//       time += period.toSec();

//       if (time == 0.0) {
//         initial_pose = robot_state.O_T_EE_c;
//         Eigen::Matrix4d m;// = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());  
//         for (int i = 0; i < 16; ++i)
//            m.data()[i] = initial_pose[i];
      
//         initial_transform = m;
//         initial_pos = initial_transform.translation();
//         initial_orientation = Eigen::Quaterniond(initial_transform.linear());
//         std::cout<<"initial orientation:"<<initial_transform.linear().eulerAngles(0,1,2).transpose()<<std::endl;

//         target_rotation = target_rotation * initial_orientation;
//       }

//       double angle = M_PI / 4 * (1 - std::cos(M_PI / target_time * time));
//       double delta_x = target[0] * std::sin(angle);
//       double delta_y = target[1] * std::sin(angle);
//       double delta_z = target[2] * std::sin(angle);
//       Eigen::Quaterniond q = initial_orientation.slerp(sin(angle), target_rotation);

//       std::array<double, 16> new_pose = initial_pose;
//       Eigen::Affine3d transform = initial_transform;
//       transform.linear() =  q.normalized().toRotationMatrix();
//       transform.translation() = initial_transform.translation()  + Eigen::Vector3d(delta_x, delta_y, delta_z);
//      // std::cout<<"before:"<<new_pose[12]<<" "<<new_pose[13]<<" "<<new_pose[14] << std::endl;
//       std::copy_n(transform.data(), new_pose.size(), new_pose.begin());
//       // new_pose[12] = initial_pose[12] + delta_x;
//       // new_pose[13] = initial_pose[13] + delta_y;
//       // new_pose[14] = initial_pose[14] + delta_z;
       
// //       std::cout<<"after:"<<new_pose[12]<<" "<<new_pose[13]<<" "<<new_pose[14] << std::endl;

//       if (time >= target_time) {
//         std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
//         std::cout << "final position:" << robot_state.O_T_EE_c[12] - initial_pose[12] << " "
//                   << robot_state.O_T_EE_c[13] - initial_pose[13] << " "
//                   << robot_state.O_T_EE_c[14] - initial_pose[14] << std::endl;

//         std::cout<<target_rotation.toRotationMatrix().eulerAngles(0,1,2).transpose()<<std::endl;

//         return franka::MotionFinished(new_pose);
//       }
//       return new_pose;
//     });

  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
