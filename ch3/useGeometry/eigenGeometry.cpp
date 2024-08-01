#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iomanip>
/****************************
* This program demonstrates the usage of the Eigen geometry module
****************************/

int main() {
    using std::cout;
    using std::endl;

    // Eigen/Geometry module provides various representations for rotations and translations
    // 3D rotation matrix using Matrix3d or Matrix3f
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();

    // Rotation vector using AngleAxis, not directly a Matrix but can be treated as one
    Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d(0, 0, 1));  // Rotate 45 degrees around Z axis

    cout << std::fixed << std::setprecision(3);
    cout << "Rotation matrix =\n" << rotation_vector.matrix() << endl;  // Convert to matrix using matrix()

    // Assign directly to rotation_matrix
    rotation_matrix = rotation_vector.toRotationMatrix();

    // Coordinate transformation using AngleAxis
    Eigen::Vector3d v(1, 0, 0);
    Eigen::Vector3d v_rotated = rotation_vector * v;
    cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;

    // Or using the rotation matrix
    v_rotated = rotation_matrix * v;
    cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;

    // Euler angles: Convert rotation matrix to Euler angles
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);  // ZYX order: roll pitch yaw
    cout << "Yaw pitch roll = " << euler_angles.transpose() << endl;

    // Euclidean transformation matrix using Eigen::Isometry
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();  // Although named 3d, it's a 4x4 matrix
    T.rotate(rotation_vector);  // Rotate according to rotation_vector
    T.pretranslate(Eigen::Vector3d(1, 3, 4));  // Set translation vector to (1, 3, 4)
    cout << "Transform matrix = \n" << T.matrix() << endl;

    // Coordinate transformation using transformation matrix
    Eigen::Vector3d v_transformed = T * v;  // Equivalent to R*v + t
    cout << "v transformed = " << v_transformed.transpose() << endl;

    // For affine and projective transformations, use Eigen::Affine3d and Eigen::Projective3d

    // Quaternions
    // Assign AngleAxis to Quaternion and vice versa
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);
    cout << "Quaternion = \n" << q.coeffs() << endl;  // Note the order is (x, y, z, w) where w is the real part

    // Assign rotation matrix to Quaternion
    q = Eigen::Quaterniond(rotation_matrix);
    cout << "Quaternion = \n" << q.coeffs() << endl;

    // Use Quaternion to rotate a vector
    v_rotated = q * v;  // Note: mathematically it's q * v * q^{-1}
    cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;

    return 0;
}
