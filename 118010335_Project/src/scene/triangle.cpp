#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
  p1 = mesh->positions[v1];
  p2 = mesh->positions[v2];
  p3 = mesh->positions[v3];
  n1 = mesh->normals[v1];
  n2 = mesh->normals[v2];
  n3 = mesh->normals[v3];
  bbox = BBox(p1);
  bbox.expand(p2);
  bbox.expand(p3);

  bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

bool Triangle::has_intersection(const Ray &r) const {
  // Part 1, Task 3: implement ray-triangle intersection
  // The difference between this function and the next function is that the next
  // function records the "intersection" while this function only tests whether
  // there is a intersection.

  // Moller-Trumbore Method
  Vector3D vecE1 = p2 - p1;
  Vector3D vecE2 = p3 - p1;
  Vector3D vecT = r.o - p1;
  Vector3D vecP = cross(r.d, vecE2);
  Vector3D vecQ = cross(vecT, vecE1);
  double dotPE1 = dot(vecP, vecE1);
  if (dotPE1 == 0)
  {
    return false;
  }
  double dotPE1Inv = 1.0 / dotPE1;
  double t = dotPE1Inv * dot(vecQ, vecE2);
  if (t < r.min_t || t > r.max_t)
  {
    return false;
  }
  double u = dotPE1Inv * dot(vecP, vecT);
  if (u < 0 || u > 1)
  {
    return false;
  }
  double v = dotPE1Inv * dot(vecQ, r.d);
  if (v < 0 || u + v > 1)
  {
    return false;
  }
	r.max_t = t;
  return true;
}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly

  // Moller-Trumbore Method
  Vector3D vecE1 = p2 - p1;
  Vector3D vecE2 = p3 - p1;
  Vector3D vecT = r.o - p1;
  Vector3D vecP = cross(r.d, vecE2);
  Vector3D vecQ = cross(vecT, vecE1);
  double dotPE1 = dot(vecP, vecE1);
  double tolerance = 0.1;
  // if (-tolerance < dotPE1 && dotPE1 < tolerance)
  if (dotPE1 == 0)
  {
    return false;
  }
  double dotPE1Inv = 1.0 / dotPE1;
  double t = dotPE1Inv * dot(vecQ, vecE2);
  if (t < r.min_t || t > r.max_t)
  {
    return false;
  }
  if (t > isect->t)
  {
    return false;
  }
  double u = dotPE1Inv * dot(vecP, vecT);
  if (u < 0 || u > 1)
  {
    return false;
  }
  double v = dotPE1Inv * dot(vecQ, r.d);
  if (v < 0 || u + v > 1)
  {
    return false;
  }
  double w = 1.0 - u - v;
	r.max_t = t;

  // Update isect
  
	isect->t = t;

  Vector3D normal = u * n1 + v * n2 + w * n3;
  // r.d points inside the surface
  // normal should point outside the surface
  // if (dot(normal, r.d) < 0)
  // {
  //   isect->n = normal;
  // }
  // else
  // {
  //   isect->n = -normal;
  // }
  isect->n = normal;

	isect->primitive = this;
	
	isect->bsdf = bsdf;

  return true;
}

void Triangle::draw(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

} // namespace SceneObjects
} // namespace CGL
