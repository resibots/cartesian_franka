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
        robot.init();
    }
    catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}
