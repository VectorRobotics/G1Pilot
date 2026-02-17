#include <gtest/gtest.h>
#include <fstream>
#include "arm_mp/motion_planner.h"  // adjust path if needed

using namespace MP;

class EE_MotionPlannerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Code here will run before each test
        obj = new MP::EE_MotionPlanner(5);
    }

    void TearDown() override {
        // Code here will run after each test
        delete obj;
    }

    MP::EE_MotionPlanner* obj;
};
// ----------------------------------------------------------------
// Test cases
// ----------------------------------------------------------------

// Test only start and goal positions
TEST_F(EE_MotionPlannerTest, ConstructLinePositionsOnly) {
    Eigen::VectorXd start(3);
    start << 0, 0, 0;
    Eigen::VectorXd goal(3);
    goal << 1, 2, 3;
    int steps = 5;

    std::vector<Eigen::VectorXd> line = obj->construct_line_(start, goal, nullptr, nullptr, nullptr, nullptr, steps);

    // Export for visualization
    std::ofstream out("line_positions.csv");
    out << "x,y,z\n";
    for (auto &p : line) {
        out << p(0) << "," << p(1) << "," << p(2) << "\n";
    }
    out.close();

    ASSERT_EQ(line.size(), steps);
    EXPECT_TRUE(line.front().isApprox(start, 1e-9));
    EXPECT_TRUE(line.back().isApprox(goal, 1e-9));

}

// Test with velocity constraints
TEST_F(EE_MotionPlannerTest, ConstructLineWithVelocity) {
    Eigen::Vector3d start(0,0,0), goal(1,1,1);
    Eigen::Vector3d start_vel(1,0,0), goal_vel(0,1,0);
    int steps = 5;

    std::vector<Eigen::VectorXd> line = obj->construct_line_(start, goal, &start_vel, &goal_vel, nullptr, nullptr, steps);

    // Export line and velocities
    std::ofstream out("line_velocities.csv");
    out << "x,y,z\n";
    for (int i = 0; i < steps; ++i) {
        auto &p = line[i];
        out << p(0) << "," << p(1) << "," << p(2) << "\n";
    }
    out.close();

    ASSERT_EQ(line.size(), steps);
    EXPECT_TRUE(line.front().isApprox(start, 1e-9));
    EXPECT_TRUE(line.back().isApprox(goal, 1e-9));

}

// Test with full constraints: positions, velocities, accelerations
TEST_F(EE_MotionPlannerTest, ConstructLineWithAllConstraints) {
    Eigen::Vector3d start(0,0,0), goal(1,1,0);
    Eigen::Vector3d start_vel(1,0,0), goal_vel(0,0,1);
    Eigen::Vector3d start_acc(0,0,1), goal_acc(1,0,0);
    int steps = 10;

    std::vector<Eigen::VectorXd> line = obj->construct_line_(start, goal, &start_vel, &goal_vel, &start_acc, &goal_acc, steps);

    // Export to CSV for Python visualization
    std::ofstream out("line_full.csv");
    out << "x,y,z\n";
    for (int i = 0; i < steps; ++i) {
        auto &p = line[i];
        out << p(0) << "," << p(1) << "," << p(2) << "\n";
    }
    out.close();
    ASSERT_EQ(line.size(), steps);
    EXPECT_TRUE(line.front().isApprox(start, 1e-9));
    EXPECT_TRUE(line.back().isApprox(goal, 1e-9));
}

// ----------------------------------------------------------------
// Main for running tests
// ----------------------------------------------------------------
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
