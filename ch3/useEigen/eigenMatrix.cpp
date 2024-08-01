#include <iostream>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>

constexpr int MATRIX_SIZE = 50;

/****************************
 * This program demonstrates the basic usage of Eigen types
 ****************************/

int main() {
    using namespace std;
    using namespace Eigen;

    // Declare a 2x3 float matrix
    Matrix<float, 2, 3> matrix_23;
    // Declare a 3-dimensional double vector
    Vector3d v_3d;
    // Declare a 3-dimensional float vector
    Matrix<float, 3, 1> vd_3d;

    // Initialize a 3x3 double matrix to zero
    Matrix3d matrix_33 = Matrix3d::Zero();
    // Dynamic size matrix declaration
    MatrixXd matrix_dynamic;
    // Simplified declaration for dynamic size matrix
    MatrixXd matrix_x;

    // Matrix operations
    // Input data
    matrix_23 << 1, 2, 3, 4, 5, 6;
    // Output matrix
    cout << "matrix_23:\n" << matrix_23 << "\n\n";

    // Accessing matrix elements
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 3; ++j)
            cout << matrix_23(i, j) << "\t";
        cout << "\n";
    }

    // Matrix and vector multiplication
    v_3d << 3, 2, 1;
    vd_3d << 4, 5, 6;
    Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
    cout << "result:\n" << result << "\n\n";

    Matrix<float, 2, 1> result2 = matrix_23 * vd_3d;
    cout << "result2:\n" << result2 << "\n\n";

    // Matrix operations
    matrix_33 = Matrix3d::Random();
    cout << "matrix_33:\n" << matrix_33 << "\n\n";
    cout << "Transpose:\n" << matrix_33.transpose() << "\n";
    cout << "Sum: " << matrix_33.sum() << "\n";
    cout << "Trace: " << matrix_33.trace() << "\n";
    cout << "10 * matrix_33:\n" << 10 * matrix_33 << "\n";
    cout << "Inverse:\n" << matrix_33.inverse() << "\n";
    cout << "Determinant: " << matrix_33.determinant() << "\n\n";

    // Eigenvalues and eigenvectors
    SelfAdjointEigenSolver<Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
    cout << "Eigenvalues:\n" << eigen_solver.eigenvalues() << "\n";
    cout << "Eigenvectors:\n" << eigen_solver.eigenvectors() << "\n\n";

    // Solving equations
    Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN = MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    Matrix<double, MATRIX_SIZE, 1> v_Nd = MatrixXd::Random(MATRIX_SIZE, 1);

    auto start = chrono::high_resolution_clock::now();
    Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    auto end = chrono::high_resolution_clock::now();
    cout << "Time used in normal inverse: " 
         << chrono::duration_cast<chrono::milliseconds>(end - start).count() << " ms\n";

    // QR decomposition
    start = chrono::high_resolution_clock::now();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    end = chrono::high_resolution_clock::now();
    cout << "Time used in QR decomposition: " 
         << chrono::duration_cast<chrono::milliseconds>(end - start).count() << " ms\n";

    return 0;
}
