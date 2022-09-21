#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.

  // The distance vector from the origin of the ray to the origin of the sphere
	Vector3D o2o = o - r.o;
  // Projection of o2o onto the ray
	double projection = dot(o2o, r.d);
  double projection_sqr = projection * projection;
  // Tangent of the sphere from the origin of the ray
	double tangent = o2o.norm2() - this->r * this->r;
  // If projection <= tangent, the ray does not have 2 intersections with the sphere
	if (projection_sqr <= tangent)
  {
		return false;
  }
  // The small right triangle formed by t1, t2, and o
  double height = o2o.norm2() - projection_sqr;
  double offset = sqrt(this->r * this->r - height);
	t1 = projection - offset;
	t2 = projection + offset;
	return true;

}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.

  double t1, t2;
	if (!test(r, t1, t2))
  {
		return false;
  }
  // t2 < min_t: false
  if (t2 < r.min_t)
  {
    return false;
  }
  // t1 < min_t <= t2 < max_t: true
  else if (t1 < r.min_t && r.min_t <= t2 && t2 < r.max_t) 
  {
    r.max_t = t2;
    return true;
  }
  // t1 < min_t < max_t < t2: false
  else if (t1 < r.min_t && r.max_t < t2)
  {
    return false;
  }
  // min_t <= t1 < max_t: true
	else if (r.min_t <= t1 && t1 < r.max_t)
  {
		r.max_t = t1;
		return true;
	}
  // max_t <= t1: false
	else
  {
		return false;
  }
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.

  double t1, t2;
	if (!test(r, t1, t2))
  {
		return false;
  }
  // t2 < min_t: false
  if (t2 < r.min_t)
  {
    return false;
  }
  // t1 < min_t <= t2 < max_t: true
  else if (t1 < r.min_t && r.min_t <= t2 && t2 < r.max_t) 
  {
    r.max_t = t2;
  }
  // t1 < min_t < max_t < t2: false
  else if (t1 < r.min_t && r.max_t < t2)
  {
    return false;
  }
  // min_t <= t1 < max_t: true
	else if (r.min_t <= t1 && t1 < r.max_t)
  {
		r.max_t = t1;
	}
  // max_t <= t1: false
	else
  {
		return false;
  }
  // Update i
	i->t = r.max_t;
	i->primitive = this;
	i->bsdf = object->get_bsdf();
  // Intersection: r.o + r.max_t * r.d
	i->n = (r.o + r.max_t * r.d - o) / this->r;
  return true;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
