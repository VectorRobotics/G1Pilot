#include <g1_pilot/g1_pilot.h>

#include <iostream>


using namespace ArmPilot;

Eigen::Matrix4d create_se3(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    transform.block<3, 1>(0, 3) = t;
    return transform;
}

// Helper function to create SE3 from quaternion components and translation
Eigen::Matrix4d create_se3(double qw, double qx, double qy, double qz, 
                           double tx, double ty, double tz) {
    Eigen::Quaterniond q(qw, qx, qy, qz);
    Eigen::Vector3d t(tx, ty, tz);
    return create_se3(q, t);
}


int main(){


    try{
        auto handle = G1DualArm();
    }
    catch (const std::exception& e) {
        std::cerr << "Exception during initialization: " << e.what() << std::endl;
        return -1;
    }

    try{
        Eigen::Matrix4d left_target = create_se3(0.0, 1.0, 0.0, 0, 0.2, 0.2, 0.1);
        Eigen::Matrix4d right_target = create_se3(0.0, 1.0, 0.0, 0, 0.2, -0.2, 0.1);
        Eigen::VectorXd current_config = Eigen::VectorXd::Constant(14, 0.1);

        auto handle = G1DualArm();

        #include <chrono>

        auto start = std::chrono::high_resolution_clock::now();
        auto result = handle.controller->control_left_arm(current_config, right_target);
        auto end = std::chrono::high_resolution_clock::now();

        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        std::cout << "control time: " << elapsed_us << " us" << std::endl;
        std::cout << "Control solution tau: " << result.transpose() << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Exception during computing control: " << e.what() << std::endl;
        return -1;
    }
}


