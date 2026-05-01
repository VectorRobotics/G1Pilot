#include <g1_pilot/g1_pilot.h>

#include <iostream>


using namespace ArmPilot;
int main(){

    try{
        Eigen::Matrix4d target = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d initial = Eigen::Matrix4d::Identity();
        target.block<3,1>(0,3) = Eigen::Vector3d(0.3,-0.149,0.095);
        initial.block<3,1>(0,3) = Eigen::Vector3d(0.21, -0.149, 0.095);

        auto handle = G1DualArm();

        std::cout <<"Arm Initialized" <<std::endl;
        std::cout <<"Initial" << initial <<std::endl;
        std::cout <<"target" << target  <<std::endl;

        std::vector<Eigen::MatrixXd> path = handle.motion_planner->planTrajectory(
            new Eigen::MatrixXd(target),
            new Eigen::MatrixXd(initial)
        );

        for (auto a: path) std::cout<<a.block<3,1>(0,3).transpose()<<std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Exception during IK solving: " << e.what() << std::endl;
        return -1;
    }
}