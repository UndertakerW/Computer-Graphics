#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out(0, 0, 0);

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading 


  
  for (int i = 0; i < num_samples; ++i)
  {
    const Vector3D w_in = hemisphereSampler->get_sample();

    Ray rBounce(hit_p, o2w * w_in, 100 * r.max_t);
    rBounce.min_t = EPS_F;
    Intersection isectBounce;
    bool bounceHit = bvh->intersect(rBounce, &isectBounce);;
    if (bounceHit)
    {
      Vector3D L_i = zero_bounce_radiance(rBounce, isectBounce);
      if (L_i.norm2() > 0)
      {
        Vector3D f_r = isect.bsdf->f(w_out, w_in);
        // cos_theta = dot(n, w_in) = w_in.z
        double cos_theta = w_in.z;
        double p_wj = 1.0 / (2.0 * PI);
        L_out += f_r * L_i * cos_theta / p_wj;
      }
    }
    
  }

  L_out *= 1.0 / num_samples;
  // L_out += zero_bounce_radiance(r, isect);
  return L_out;

}

Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  Vector3D L_out(0, 0, 0);

  int sample_count = 0;
  for (SceneLight* light : scene->lights) 
  {
    Vector3D radiance;
    Vector3D wi;
    double distToLight;
    double pdf;

    int num_light_samples = ns_area_light;
    if (light->is_delta_light())
    {
      num_light_samples = 1;
    }
    // integrate light over the hemisphere about the normal
    for (int i = 0; i < num_light_samples; i++) 
    {
      // Sample the light
      Vector3D L_light = light->sample_L(hit_p, &wi, &distToLight, &pdf);
      Vector3D w_in = w2o * wi;
      // If the light is under the surface, discard
      if (w_in.z < 0)
      {
        continue;
      }
      Ray shadowRay(hit_p, wi);
      // Avoid hitting the surface containing hit_p
      // Avoid hitting the light itself
      shadowRay.min_t = EPS_F;
			shadowRay.max_t = distToLight - EPS_F;
      Intersection isect_shadow;
			if (!bvh->intersect(shadowRay, &isect_shadow))
      {
        Vector3D f_r = isect.bsdf->f(w_out, w_in);
        // cos_theta = dot(n, w_in) = w_in.z
        double cos_theta = w_in.z;
        radiance += L_light * f_r * cos_theta / pdf;
      }
    }
    L_out += radiance * (1.0 / (double) num_light_samples);
	}
  // L_out += zero_bounce_radiance(r, isect);
  return L_out;

}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light

  Vector3D emission = isect.bsdf->get_emission();

  return emission;

}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`

  Vector3D L_out;
  if (direct_hemisphere_sample)
  {
    L_out = estimate_direct_lighting_hemisphere(r, isect);
  }
  else
  {
    L_out = estimate_direct_lighting_importance(r, isect);
  }
  return L_out;

}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {

  if (max_ray_depth == 0)
  {
    return Vector3D(0, 0, 0);
  }             

  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  Vector3D L_out(0, 0, 0);
  if (!isect.bsdf->is_delta())
  {
    L_out = one_bounce_radiance(r, isect);
  }

  // TODO: Part 4, Task 2
  // Returns the one bounce radiance + radiance from extra bounces at this point.
  // Should be called recursively to simulate extra bounces.

  if (max_ray_depth == 1)
  {
    return L_out;
  }

  double pRoulette = 0.35;

  // Always trace at least one indirect bounce regardless of Russian Roulette
  if ((coin_flip(pRoulette) && r.depth > 1) || r.depth == max_ray_depth)
  {
    // Sample only one ray for indirect lighting
    Vector3D w_in;
    double pdf; 
    Vector3D f_r = isect.bsdf->sample_f(w_out, &w_in, &pdf);
    // cos_theta = dot(n, w_in) = w_in.z
    double cos_theta = w_in.z;
    Ray rBounce(hit_p, o2w * w_in, (int) r.depth - 1);
    rBounce.min_t = EPS_F;
    Intersection isectBounce;
    if (bvh->intersect(rBounce, &isectBounce))
    {
      Vector3D L_bounce = at_least_one_bounce_radiance(rBounce, isectBounce);
      if (isect.bsdf->is_delta())
      {
        L_bounce += zero_bounce_radiance(rBounce, isectBounce);
      }
      // If Russian Roulette is used
      if (r.depth != max_ray_depth)
      {
        L_out += f_r * L_bounce * cos_theta / pdf / pRoulette;
      }
      else
      {
        L_out += f_r * L_bounce * cos_theta / pdf;
      }
    }
  }

  return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.
  //
  // REMOVE THIS LINE when you are ready to begin Part 3.
  
  if (!bvh->intersect(r, &isect))
  {
    return envLight ? envLight->sample_dir(r) : L_out;
  }

  // L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);

  // TODO (Part 3): Return the direct illumination.

  // L_out = zero_bounce_radiance(r, isect);
  // L_out = one_bounce_radiance(r, isect);

  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct
  L_out = zero_bounce_radiance(r, isect) + at_least_one_bounce_radiance(r, isect);
  //L_out = at_least_one_bounce_radiance(r, isect) - one_bounce_radiance(r, isect);

  return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  // TODO (Part 1.2):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Vector3D.
  // You should call est_radiance_global_illumination in this function.

	Vector3D color(0, 0, 0);
  double s1 = 0;
  double s2 = 0;
  size_t ns_adaptive = 0;
	for (int i = 0; i < ns_aa; i++) 
  {
		Vector2D samplePoint = gridSampler->get_sample();
    //Vector2D samplePoint(0.5, 0.5);
		double xNorm = ((double) x + samplePoint.x) / sampleBuffer.w;
		double yNorm = ((double) y + samplePoint.y) / sampleBuffer.h;
    Ray r = camera->generate_ray(xNorm, yNorm);
    r.depth = max_ray_depth;
		Vector3D sampleColor = est_radiance_global_illumination(r);
    color += sampleColor;
    
    //Adapative Sampling
    ns_adaptive++;
    if (samplesPerBatch > 0)
    {
      double illum = sampleColor.illum();
      s1 += illum;
      s2 += illum * illum;
      if ((i + 1) % samplesPerBatch == 0)
      {
        double mean = s1 / (i + 1);
        double stdDev = sqrt(1.0 / i * (s2 - (s1 * s1) / (i + 1)));
        if (1.96 * stdDev / sqrt(i + 1) <= maxTolerance * mean) 
        {
          break;
        }
      }
    }
    
	}
	color *= 1.0 / ns_adaptive;
  // sampleBuffer.update_pixel(color, x, y);
  // return;

  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"

  int num_samples = ns_adaptive;          // total samples to evaluate
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel


  sampleBuffer.update_pixel(color, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;


}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
