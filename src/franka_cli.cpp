// Copyright (c) 2021 Inria

#include <cassert>
#include <cmath>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <franka/exception.h>
#include <franka/robot.h>

#include "cartesian_franka/robot.hpp"

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }
    try {
        cartesian_franka::Robot robot(argv[1]);
        std::cout<<"Starting position:" << robot.position().transpose() << std::endl;
        std::cout<<"Starting Euler angles:" << robot.orientation().transpose() << std::endl;
        std::cout<<"Moving to the init position..." << std::endl;

        robot.init();

        std::cout<<"Init position:" << robot.position().transpose() << std::endl;
        std::cout<<"Init Euler angles:" << robot.orientation().transpose() << std::endl;

        robot.translate(Eigen::Vector3d(0.1, 0, 0));

    }
    catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}
