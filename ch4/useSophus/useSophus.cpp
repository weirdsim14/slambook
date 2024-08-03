#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

// Overload the << operator for Sophus::SO3d
std::ostream& operator<<(std::ostream& out, const Sophus::SO3d& so3) {
    Eigen::Quaterniond q = so3.unit_quaternion();
    out << "SO3 [w, x, y, z]: [" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "]";
    return out;
}

// Overload the << operator for Sophus::SE3d
std::ostream& operator<<(std::ostream& out, const Sophus::SE3d& se3) {
    Eigen::Quaterniond q = se3.unit_quaternion();
    Eigen::Vector3d t = se3.translation();
    out << "SE3 [translation: " << t.transpose() << ", quaternion: " << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "]";
    return out;
}

int main() {
    // Rotation matrix for a 90 degree rotation around Z axis
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
    
    Sophus::SO3d SO3_R(R);                // Construct from rotation matrix
    Sophus::SO3d SO3_v = Sophus::SO3d::exp(Eigen::Vector3d(0, 0, M_PI / 2));   // Construct from rotation vector
    Eigen::Quaterniond q(R);              // Construct from quaternion
    Sophus::SO3d SO3_q(q);
    
    // All representations are equivalent
    std::cout << "SO(3) from matrix: " << SO3_R << std::endl;
    std::cout << "SO(3) from vector: " << SO3_v << std::endl;
    std::cout << "SO(3) from quaternion: " << SO3_q << std::endl;
    
    // Logarithmic map to get the Lie algebra
    Eigen::Vector3d so3 = SO3_R.log();
    std::cout << "so3 = " << so3.transpose() << std::endl;
    // Hat operator to convert vector to skew-symmetric matrix
    std::cout << "so3 hat =\n" << Sophus::SO3d::hat(so3) << std::endl;
    // Vee operator to convert skew-symmetric matrix back to vector
    std::cout << "so3 hat vee = " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << std::endl;

    // Update model with a small perturbation
    Eigen::Vector3d update_so3(1e-4, 0, 0); // Small update
    Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;
    std::cout << "SO3 updated = " << SO3_updated << std::endl;

    std::cout << "************ Separator *************" << std::endl;

    // SE(3) operations
    Eigen::Vector3d t(1, 0, 0);           // Translation vector along X axis
    Sophus::SE3d SE3_Rt(R, t);            // Construct SE(3) from rotation matrix and translation
    Sophus::SE3d SE3_qt(q, t);            // Construct SE(3) from quaternion and translation
    std::cout << "SE3 from R,t= " << std::endl << SE3_Rt << std::endl;
    std::cout << "SE3 from q,t= " << std::endl << SE3_qt << std::endl;
    
    // Lie algebra se(3) is a six-dimensional vector
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    Vector6d se3 = SE3_Rt.log();
    std::cout << "se3 = " << se3.transpose() << std::endl;
    // Hat and vee operators
    std::cout << "se3 hat = " << std::endl << Sophus::SE3d::hat(se3) << std::endl;
    std::cout << "se3 hat vee = " << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << std::endl;

    // Update SE(3) with a small perturbation
    Vector6d update_se3 = Vector6d::Zero();
    update_se3(0, 0) = 1e-4;
    Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Rt;
    std::cout << "SE3 updated = " << std::endl << SE3_updated.matrix() << std::endl;

    return 0;
}
