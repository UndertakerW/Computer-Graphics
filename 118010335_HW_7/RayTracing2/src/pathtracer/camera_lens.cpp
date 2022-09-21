#include "camera.h"

#include <iostream>
#include <sstream>
#include <fstream>

#include "CGL/misc.h"
#include "CGL/vector2D.h"
#include "CGL/vector3D.h"

using std::cout;
using std::endl;
using std::max;
using std::min;
using std::ifstream;
using std::ofstream;

namespace CGL {

using Collada::CameraInfo;

Ray Camera::generate_ray_for_thin_lens(double x, double y, double rndR, double rndTheta) const {

  // Part 2, Task 4:
  // compute position and direction of ray from the input sensor sample coordinate.
  // Note: use rndR and rndTheta to uniformly sample a unit disk.

  // Calculate width and height of the sensor
  double sensorHalfW = tan(radians(hFov / 2));
	double sensorHalfH = tan(radians(vFov / 2));
  double sensorW = 2 * sensorHalfW;
	double sensorH = 2 * sensorHalfH;

  // In the camera space
	Vector3D pFilm(-sensorHalfW + x * sensorW, -sensorHalfH + y * sensorH, -1.0);

  Vector3D dirPFilmToCenter = Vector3D(0, 0, 0) - pFilm;
  Ray rCenter = Ray(Vector3D(0, 0, 0), dirPFilmToCenter);

  double tPFocus = - focalDistance / dirPFilmToCenter.z;
  Vector3D pFocal = rCenter.at_time(tPFocus);

  Vector3D pLens = Vector3D(lensRadius * sqrt(rndR) * cos(rndTheta), lensRadius * sqrt(rndR) * sin(rndTheta), 0);

  Vector3D dirPLensToPFocal = pFocal - pLens;
  dirPLensToPFocal.normalize();

  Vector3D pLensWorld = pos + c2w * pLens;
  Vector3D dirPLensToPFocalWorld = c2w * dirPLensToPFocal;

  Ray rSample = Ray(pLensWorld, dirPLensToPFocalWorld);
  rSample.min_t = nClip;
  rSample.max_t = fClip;

  return rSample;
}


} // namespace CGL
