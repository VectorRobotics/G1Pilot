#include <arm_mp/motion_planner.h>
#include <gtest/gtest.h>

namespace MP {

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

// Example test for method1()
TEST_F(EE_MotionPlannerTest, generate_pow_vector_1test) {
    // Arrange
    double s0 = 0.0, s5 = 0.5, s1 = 1.0;
    int pow0 = 0, pow1 = 1, pow3 = 3;

    // Act & Assert

    // s0, pow0 -> [1]
    Eigen::VectorXd result = obj->generate_pow_vector_(s0, pow0);
    Eigen::VectorXd expected(1);
    expected << 1.0;
    EXPECT_EQ(result, expected);

    // s5, pow0 -> [1]
    result = obj->generate_pow_vector_(s5, pow0);
    expected << 1.0;
    EXPECT_EQ(result, expected);

    // s0, pow1 -> [1, 0]
    result = obj->generate_pow_vector_(s0, pow1);
    expected.resize(2);
    expected << 1.0, 0.0;
    EXPECT_EQ(result, expected);

    // s5, pow1 -> [1, 0.5]
    result = obj->generate_pow_vector_(s5, pow1);
    expected << 1.0, 0.5;
    EXPECT_EQ(result, expected);

    // s0, pow3 -> [1, 0, 0, 0]
    result = obj->generate_pow_vector_(s0, pow3);
    expected.resize(4);
    expected << 1.0, 0.0, 0.0, 0.0;
    EXPECT_EQ(result, expected);

    // s5, pow3 -> [1, 0.5, 0.25, 0.125]
    result = obj->generate_pow_vector_(s5, pow3);
    expected << 1.0, 0.5, 0.25, 0.125;
    EXPECT_EQ(result, expected);

    // s1, pow3 -> [1, 1, 1, 1]
    result = obj->generate_pow_vector_(s1, pow3);
    expected << 1.0, 1.0, 1.0, 1.0;
    EXPECT_EQ(result, expected);
}


TEST_F(EE_MotionPlannerTest, generate_pow_vector_matrix_test) {
    // Arrange
    Eigen::VectorXd s(3);
    s << 0.0, 0.5, 1.0;

    int pow0 = 0, pow1 = 1, pow2 = 2, pow3 = 3;

    Eigen::MatrixXd result, expected;

    // pow = 0 -> 1 row
    result = obj->generate_pow_vector_(s, pow0);
    expected.resize(1, 3);
    expected << 1.0, 1.0, 1.0;
    EXPECT_TRUE(result.isApprox(expected));

    // pow = 1 -> 2 rows
    result = obj->generate_pow_vector_(s, pow1);
    expected.resize(2, 3);
    expected << 1.0, 1.0, 1.0,
                0.0, 0.5, 1.0;
    EXPECT_TRUE(result.isApprox(expected));

    // pow = 2 -> 3 rows
    result = obj->generate_pow_vector_(s, pow2);
    expected.resize(3, 3);
    expected << 1.0, 1.0, 1.0,
                0.0, 0.5, 1.0,
                0.0, 0.25, 1.0;
    EXPECT_TRUE(result.isApprox(expected));

    // pow = 3 -> 4 rows
    result = obj->generate_pow_vector_(s, pow3);
    expected.resize(4, 3);
    expected << 1.0, 1.0, 1.0,
                0.0, 0.5, 1.0,
                0.0, 0.25, 1.0,
                0.0, 0.125, 1.0;
    EXPECT_TRUE(result.isApprox(expected));
}


TEST_F(EE_MotionPlannerTest, evaluate_polynomial_test) {
    // Arrange
    Eigen::VectorXd t(3);
    t << 0.0, 1.0, 2.0;

    Eigen::MatrixXd coeff(2, 3); // 2 polynomials, degree 2
    coeff << 1, 2, 3,    // p0(t) = 1 + 2 t + 3 t^2
             0, 1, -1;   // p1(t) = 0 + 1 t - 1 t^2

    // Act
    std::vector<Eigen::VectorXd> result = obj->evaluate_polynomial(t, coeff);

    // Assert
    ASSERT_EQ(result.size(), 3); // 3 time steps

    Eigen::VectorXd expected0(2); expected0 << 1, 0;
    Eigen::VectorXd expected1(2); expected1 << 6, 0;
    Eigen::VectorXd expected2(2); expected2 << 17, -2;

    EXPECT_TRUE(result[0].isApprox(expected0));
    EXPECT_TRUE(result[1].isApprox(expected1));
    EXPECT_TRUE(result[2].isApprox(expected2));
}

TEST_F(EE_MotionPlannerTest, diff_matrix_test) {
    int pol_order = 3;

    // 0th derivative -> Identity
    Eigen::MatrixXd result = obj->diff_(pol_order, 0);
    Eigen::MatrixXd expected = Eigen::MatrixXd::Identity(pol_order+1, pol_order+1);
    EXPECT_TRUE(result.isApprox(expected));

    // 1st derivative
    result = obj->diff_(pol_order, 1);
    expected.resize(pol_order+1, pol_order);
    expected << 0, 0, 0,
                1, 0, 0,
                0, 2, 0,
                0, 0, 3;
    EXPECT_TRUE(result.isApprox(expected));

    // 2nd derivative
    result = obj->diff_(pol_order, 2);
    expected.resize(pol_order+1, pol_order-1); // 4x2
    expected << 0, 0,
                0, 0,
                2, 0,
                0, 6;
    EXPECT_TRUE(result.isApprox(expected));

    // 3rd derivative
    result = obj->diff_(pol_order, 3);
    expected.resize(pol_order+1, pol_order-2); // 4x1
    expected << 0,
                0,
                0,
                6;
    EXPECT_TRUE(result.isApprox(expected));
}

// TEST_F(EE_MotionPlannerTest, diff_matrix_edge_cases_test) {
//     // ==========================
//     // pol_order = 0
//     // ==========================
//     int pol_order = 0;

//     // 0th derivative -> Identity
//     Eigen::MatrixXd result = obj->diff_(pol_order, 0);
//     Eigen::MatrixXd expected = Eigen::MatrixXd::Identity(1,1);
//     EXPECT_TRUE(result.isApprox(expected));

//     // 1st derivative -> all zeros
//     result = obj->diff_(pol_order, 1);
//     expected.resize(1,1);
//     expected << 0.0;
//     EXPECT_TRUE(result.isApprox(expected));

//     // 2nd derivative -> all zeros
//     result = obj->diff_(pol_order, 2);
//     expected << 0.0;
//     EXPECT_TRUE(result.isApprox(expected));

//     // ==========================
//     // pol_order = 1
//     // ==========================
//     pol_order = 1;

//     // 0th derivative -> Identity
//     result = obj->diff_(pol_order, 0);
//     expected = Eigen::MatrixXd::Identity(2,2);
//     EXPECT_TRUE(result.isApprox(expected));

//     // 1st derivative -> 2x1 matrix, all zeros
//     result = obj->diff_(pol_order, 1);
//     expected.resize(2,1);
//     expected << 0.0,
//                 0.0;
//     EXPECT_TRUE(result.isApprox(expected));

//     // 2nd derivative -> 2x0 (but treat as zeros 2x0?) → just check entries if non-empty
//     result = obj->diff_(pol_order, 2);
//     if(result.size() > 0) {
//         EXPECT_TRUE(result.isZero(1e-12)); // all entries zero
//     }
// }

} // namespace MP

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv); // Initialize Google Test
    return RUN_ALL_TESTS();                 // Run all tests and return result
}



// You can add more TEST_F blocks for other methods
