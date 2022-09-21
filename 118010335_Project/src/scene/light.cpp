#include "light.h"
#include "bvh.h"

#include <iostream>

#include "pathtracer/sampler.h"

namespace CGL { namespace SceneObjects {

// Directional Light //

DirectionalLight::DirectionalLight(const Vector3D rad,
                                   const Vector3D lightDir)
    : radiance(rad) {
  dirToLight = -lightDir.unit();
}

Vector3D DirectionalLight::sample_L(const Vector3D p, Vector3D* wi,
                                    double* distToLight, double* pdf) const {
  *wi = dirToLight;
  *distToLight = INF_D;
  *pdf = 1.0;
  return radiance;
}

Vector3D DirectionalLight::sample_L_backward(const Vector3D d, const BVHAccel* bvh,
                             Vector3D* sample, Vector3D* hit_p, double* distToLight, double* pdf, BSDF** bsdf, Vector3D* n) const {

  // *sample = d * 10000;
  // if (*sample * -dirToLight > 0)
  // {
  //   *sample = -*sample;
  // }
  // Ray r(*sample, -dirToLight);
  // r.min_t = EPS_F;
  // Intersection isect;
  // bool hit = bvh->intersect(r, &isect);
  // if (hit)
  // {
  //   *hit_p = r.o + r.d * isect.t;
  //   double cosTheta = dot(d, direction);
  //   *distToLight = INF_D;
  //   *pdf = 1.0 / (2.0 * PI);
  //   *bsdf = isect.bsdf;

  //   return radiance;
  // }
  *pdf = 0;
  return Vector3D();

};

// Infinite Hemisphere Light //

InfiniteHemisphereLight::InfiniteHemisphereLight(const Vector3D rad)
    : radiance(rad) {
  sampleToWorld[0] = Vector3D(1,  0,  0);
  sampleToWorld[1] = Vector3D(0,  0, -1);
  sampleToWorld[2] = Vector3D(0,  1,  0);
}

Vector3D InfiniteHemisphereLight::sample_L(const Vector3D p, Vector3D* wi,
                                           double* distToLight,
                                           double* pdf) const {
  Vector3D dir = sampler.get_sample();
  *wi = sampleToWorld* dir;
  *distToLight = INF_D;
  *pdf = 1.0 / (2.0 * PI);
  return radiance;
}

Vector3D InfiniteHemisphereLight::sample_L_backward(const Vector3D d, const BVHAccel* bvh,
                             Vector3D* sample, Vector3D* hit_p, double* distToLight, double* pdf, BSDF** bsdf, Vector3D* n) const {

  *sample = d * 10000;
  Ray r(sampleToWorld * *sample, sampleToWorld * -d);
  r.min_t = EPS_F;
  Intersection isect;
  bool hit = bvh->intersect(r, &isect);
  if (hit)
  {
    *hit_p = r.o + r.d * isect.t;
    *distToLight = INF_D;
    *pdf = 1.0 / (2.0 * PI);
    *bsdf = isect.bsdf;
    *n = isect.n;

    return radiance;
  }
  *pdf = 0;
  return Vector3D();

};

// Point Light //

PointLight::PointLight(const Vector3D rad, const Vector3D pos) : 
  radiance(rad), position(pos) { }

Vector3D PointLight::sample_L(const Vector3D p, Vector3D* wi,
                             double* distToLight,
                             double* pdf) const {
  Vector3D d = position - p;
  *wi = d.unit();
  *distToLight = d.norm();
  *pdf = 1.0;
  return radiance;
}

Vector3D PointLight::sample_L_backward(const Vector3D d, const BVHAccel* bvh,
                             Vector3D* sample, Vector3D* hit_p, double* distToLight, double* pdf, BSDF** bsdf, Vector3D* n) const {

  Vector2D sample2D = sampler.get_sample();
  Vector3D ray_dir;
  if (sample2D[0] < 0.5)
  {
    ray_dir = d;
  }
  else
  {
    ray_dir = -d;
  }
  Ray r(position, ray_dir);
  r.min_t = EPS_F;
  Intersection isect;
  bool hit = bvh->intersect(r, &isect);
  if (hit)
  {
    *hit_p = r.o + r.d * isect.t;
    *distToLight = isect.t;
    *pdf = 0.5;
    *bsdf = isect.bsdf;
    *n = isect.n;

    return radiance;
  }
  *pdf = 0;
  return Vector3D();

};


// Spot Light //

SpotLight::SpotLight(const Vector3D rad, const Vector3D pos,
                     const Vector3D dir, double angle) {

}

Vector3D SpotLight::sample_L(const Vector3D p, Vector3D* wi,
                             double* distToLight, double* pdf) const {
  return Vector3D();
}

Vector3D SpotLight::sample_L_backward(const Vector3D d, const BVHAccel* bvh,
                             Vector3D* sample, Vector3D* hit_p, double* distToLight, double* pdf, BSDF** bsdf, Vector3D* n) const {
  return Vector3D();
}


// Area Light //

AreaLight::AreaLight(const Vector3D rad, 
                     const Vector3D pos,   const Vector3D dir, 
                     const Vector3D dim_x, const Vector3D dim_y)
  : radiance(rad), position(pos), direction(dir),
    dim_x(dim_x), dim_y(dim_y), area(dim_x.norm() * dim_y.norm()) { }

Vector3D AreaLight::sample_L(const Vector3D p, Vector3D* wi, 
                             double* distToLight, double* pdf) const {

  Vector2D sample = sampler.get_sample() - Vector2D(0.5f, 0.5f);
  Vector3D d = position + sample.x * dim_x + sample.y * dim_y - p;
  double cosTheta = dot(d, direction);
  double sqDist = d.norm2();
  double dist = sqrt(sqDist);
  *wi = d / dist;
  *distToLight = dist;
  *pdf = sqDist / (area * fabs(cosTheta));
  return cosTheta < 0 ? radiance : Vector3D();
};

Vector3D AreaLight::sample_L_backward(const Vector3D d, const BVHAccel* bvh,
                             Vector3D* sample, Vector3D* hit_p, double* distToLight, double* pdf, BSDF** bsdf, Vector3D* n) const {

  Matrix3x3 o2w;
  make_coord_space(o2w, direction);
  Matrix3x3 w2o = o2w.T();

  Vector2D sample2D = sampler.get_sample() - Vector2D(0.5f, 0.5f);
  *sample = position + sample2D.x * dim_x + sample2D.y * dim_y;

  Ray r(*sample, o2w * d);
  r.min_t = EPS_F;
  Intersection isect;
  bool hit = bvh->intersect(r, &isect);
  if (hit)
  {
    *hit_p = r.o + r.d * isect.t;
    double cosTheta = dot(d, direction);
    *distToLight = isect.t;
    *pdf = (isect.t * isect.t) / (area * fabs(cosTheta));
    *bsdf = isect.bsdf;
    *n = isect.n;

    return cosTheta < 0 ? radiance : Vector3D();
  }
  *pdf = 0;
  return Vector3D();

};


// Sphere Light //

SphereLight::SphereLight(const Vector3D rad, const SphereObject* sphere) {

}

Vector3D SphereLight::sample_L(const Vector3D p, Vector3D* wi, 
                               double* distToLight, double* pdf) const {

  return Vector3D();
}

Vector3D SphereLight::sample_L_backward(const Vector3D d, const BVHAccel* bvh,
                             Vector3D* sample, Vector3D* hit_p, double* distToLight, double* pdf, BSDF** bsdf, Vector3D* n) const {
  return Vector3D();
}

// Mesh Light

MeshLight::MeshLight(const Vector3D rad, const Mesh* mesh) {

}

Vector3D MeshLight::sample_L(const Vector3D p, Vector3D* wi, 
                             double* distToLight, double* pdf) const {
  return Vector3D();
}

Vector3D MeshLight::sample_L_backward(const Vector3D d, const BVHAccel* bvh,
                             Vector3D* sample, Vector3D* hit_p, double* distToLight, double* pdf, BSDF** bsdf, Vector3D* n) const {
  return Vector3D();
}

} // namespace SceneObjects
} // namespace CGL
