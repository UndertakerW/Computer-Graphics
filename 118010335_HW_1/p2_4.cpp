#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <iostream>

using namespace Eigen;
using namespace std;

int main() {

    Vector3f v(1, 2, 3);
    Vector3f w(4, 5, 6);

    AngleAxisf rotation_vector_x(M_PI / 4, Vector3f(1, 0, 0));
    AngleAxisf rotation_vector_y(M_PI / 6, Vector3f(0, 1, 0));
    AngleAxisf rotation_vector_z(M_PI / 3, Vector3f(0, 0, 1));

    Vector3f v_rotated = (rotation_vector_z * rotation_vector_y 
        * rotation_vector_x * (v - w)) + w;

    cout << v_rotated << endl;

    return 0;
}