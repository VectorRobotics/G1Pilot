#include <g1_pilot/g1_pilot.h>

#include <iostream>


using namespace ArmPilot;
int main(){

    try{
        Eigen::Matrix4d left_target = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d right_target = Eigen::Matrix4d::Identity();
        right_target.block<3,1>(0,3) = Eigen::Vector3d(0.01,0,0);

        auto handle = G1DualArm();

        std::cout <<"Arm Initialized" <<std::endl;

        std::vector<Eigen::MatrixXd> path = handle.motion_planner->planTrajectory(
            new Eigen::MatrixXd(right_target)
        );

        for (auto pose: path){
            auto result = handle.ik->solve_ik(left_target, pose);
            auto q = Eigen::VectorXd::Map(result.position.data(), result.position.size());
            std::cout << "IK solution q: " << q.transpose() << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception during IK solving: " << e.what() << std::endl;
        return -1;
    }
}