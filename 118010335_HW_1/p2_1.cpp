#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <iostream>

using namespace Eigen;
using namespace std;

int main() {
    // problem 1
    Vector4f v(1, 1.5, 2, 3);
    Vector4f w(0, 1, 2, 4);
    Vector3f v_3d, w_3d;
    for (int i = 0; i < 3; i++) {
        v_3d(i) = v(i) / v(3);
        w_3d(i) = w(i) / w(3);
    }
    // problem 2
    cout << "[problem 2]" << endl;
    cout << v_3d + w_3d << endl;
    // problem 3
    cout << "[problem 3]" << endl;
    cout << v_3d.dot(w_3d) << endl;
    // problem 4
    cout << "[problem 4]" << endl;
    cout << v_3d.cross(w_3d) << endl;
    return 0;
}