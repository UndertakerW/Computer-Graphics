#include "transforms.h"

#include "CGL/matrix3x3.h"
#include "CGL/vector2D.h"
#include "CGL/vector3D.h"

constexpr double MY_PI = 3.1415926;

namespace CGL {

Vector2D operator*(const Matrix3x3 &m, const Vector2D &v) {
	Vector3D mv = m * Vector3D(v.x, v.y, 1);
	return Vector2D(mv.x / mv.z, mv.y / mv.z);
}

Matrix3x3 translate(float dx, float dy) {
	// Part 3: Fill this in.
	Matrix3x3 M_trans(1, 0, dx,
		0, 1, dy,
		0, 0, 1);

	return M_trans;
}

Matrix3x3 scale(float sx, float sy) {
	// Part 3: Fill this in.
	Matrix3x3 M_scale(sx, 0, 0,
		0, sy, 0,
		0, 0, 1);

	return M_scale;
}

// The input argument is in degrees counterclockwise
Matrix3x3 rotate(float deg) {
	// Part 3: Fill this in.
	float radian = deg / 180.0 * MY_PI;
	Matrix3x3 M_rot(std::cos(radian), -std::sin(radian), 0,
		std::sin(radian), std::cos(radian), 0,
		0, 0, 1);

	return M_rot;
}

}
