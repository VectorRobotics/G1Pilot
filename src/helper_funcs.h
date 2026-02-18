#include <Eigen/Dense>

Eigen::Matrix4d create_se3(const Eigen::Quaterniond &q, const Eigen::Vector3d &t)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    transform.block<3, 1>(0, 3) = t;
    return transform;
}

Eigen::Matrix4d create_se3(double qw, double qx, double qy, double qz,
                            double tx, double ty, double tz)
{
    Eigen::Quaterniond q(qw, qx, qy, qz);
    Eigen::Vector3d t(tx, ty, tz);
    return create_se3(q, t);
}