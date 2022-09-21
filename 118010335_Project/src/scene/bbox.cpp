#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.

	double tMin = r.min_t;
	double tMax = r.max_t;

  // If 2 components of r.d is 0
  // need to check whether the orthogonal projection is inside the boundary
  for (int i = 0; i < 3; ++i)
  {
    double ta, tb;
    // if r.d[i] == r.d[(i + 1) % 3] == 0
    if (r.d[(i + 2) % 3] == 1)
    {
      if (min[i] < r.o[i] && r.o[i] < max[i] && 
        min[(i + 1) % 3] < r.o[(i + 1) % 3] && r.o[(i + 1) % 3] < max[(i + 1) % 3])
      {
        if (r.d[(i + 2) % 3] > 0)
        {
          ta = (min[(i + 2) % 3] - r.o[(i + 2) % 3]) / r.d[(i + 2) % 3];
          tb = (max[(i + 2) % 3] - r.o[(i + 2) % 3]) / r.d[(i + 2) % 3];
        }
        else
        {
          tb = (min[(i + 2) % 3] - r.o[(i + 2) % 3]) / r.d[(i + 2) % 3];
          ta = (max[(i + 2) % 3] - r.o[(i + 2) % 3]) / r.d[(i + 2) % 3];
        }
        if (tMin < ta || tb < tMax)
        {
          t0 = std::max(ta, tMin);
          t1 = std::min(tb, tMax);
          return true;
        }
        return false;
      }
    }
  }

  // Else if 1 components of r.d is 0
  // Or all components are non-zero
	for (size_t i = 0; i < 3; i++)
  {
    double ta, tb;
    if (r.d[i] == 0)
    {
      continue;
    }
    if (r.d[i] > 0)
    {
      ta = (min[i] - r.o[i]) / r.d[i];
		  tb = (max[i] - r.o[i]) / r.d[i];
    }
    else
    {
      tb = (min[i] - r.o[i]) / r.d[i];
		  ta = (max[i] - r.o[i]) / r.d[i];
    }
		tMin = std::max(ta, tMin);
		tMax = std::min(tb, tMax);
    // If tMax < tMin: no intersection with the bbox
		if (tMax < tMin)
    {
      return false;
    }
	}

	t0 = tMin;
	t1 = tMax;

  return true;

}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
