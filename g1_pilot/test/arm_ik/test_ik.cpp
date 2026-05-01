#include <g1_pilot/g1_pilot.h>

#include <iostream>


using namespace HumanoidPilot;
int main(){

    std::cout << "Using CasADi for optimization." << std::endl;

    try{
        auto handle = Humanoid();
    }
    catch (const std::exception& e) {
        std::cerr << "Exception during IK solver initialization: " << e.what() << std::endl;
        return -1;
    }
    std::cout << "Init passed." << std::endl;

    try{
        Eigen::Matrix4d left_target = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d right_target = Eigen::Matrix4d::Identity();

        auto handle = Humanoid();

        #include <chrono>

        auto start = std::chrono::high_resolution_clock::now();
        auto result = handle.ik->solve_ik(left_target, right_target);
        auto end = std::chrono::high_resolution_clock::now();

        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        std::cout << "solve_ik time: " << elapsed_us << " us" << std::endl;
    std::chrono::duration<double> elapsed = end - start;
    auto q = Eigen::VectorXd::Map(result.position.data(), result.position.size());
    auto tau = Eigen::VectorXd::Map(result.effort.data(), result.effort.size());
    std::cout << "IK solve time: " << elapsed.count() << " s" << std::endl;

    std::cout << "IK solution q: " << q.transpose() << std::endl;
    std::cout << "IK solution tau: " << tau.transpose() << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Exception during IK solving: " << e.what() << std::endl;
        return -1;
    }
}