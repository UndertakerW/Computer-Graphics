#include "bsdf.h"

#include <algorithm>
#include <iostream>
#include <utility>

#include "application/visual_debugger.h"

using std::max;
using std::min;
using std::swap;

namespace CGL {

// Mirror BSDF //

Vector3D MirrorBSDF::f(const Vector3D wo, const Vector3D wi) {
  return Vector3D();
}

Vector3D MirrorBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {

  // TODO:
  // Implement MirrorBSDF
  *pdf = 1.0;
  reflect(wo, wi);
  return reflectance / abs_cos_theta(*wi);
}

void MirrorBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Mirror BSDF"))
  {
    DragDouble3("Reflectance", &reflectance[0], 0.005);
    ImGui::TreePop();
  }
}

// Microfacet BSDF //

double MicrofacetBSDF::G(const Vector3D wo, const Vector3D wi) {
  return 1.0 / (1.0 + Lambda(wi) + Lambda(wo));
}

double MicrofacetBSDF::D(const Vector3D h) {
  // TODO: proj3-2, part 3
  // Compute Beckmann normal distribution function (NDF) here.
  // You will need the roughness alpha.

  double cos_theta_h = h.z;
  double sin_theta_h = sqrt(1 - cos_theta_h * cos_theta_h);
  double tan_theta_h = sin_theta_h / cos_theta_h;
  double D = exp(-(tan_theta_h * tan_theta_h) / (alpha * alpha))
         / (PI * alpha * alpha * cos_theta_h * cos_theta_h * cos_theta_h * cos_theta_h);
  return D;
}

double MicrofacetBSDF::fresnel_term(double eta, double k, double cos_theta_i) {
  double r0 = eta * eta + k * k;
  double cos_theta_i_sqr = cos_theta_i * cos_theta_i;
  double rs = (r0 - 2.0 * eta * cos_theta_i + cos_theta_i_sqr)
            / (r0 + 2.0 * eta * cos_theta_i + cos_theta_i_sqr);
  double rp = (r0 * cos_theta_i_sqr - 2.0 * eta * cos_theta_i + 1.0)
            / (r0 * cos_theta_i_sqr + 2.0 * eta * cos_theta_i + 1.0);
  double fresnel_term = (rs + rp) * 0.5;
  return fresnel_term;
}

Vector3D MicrofacetBSDF::F(const Vector3D wi) {
  // TODO: proj3-2, part 3
  // Compute Fresnel term for reflection on dielectric-conductor interface.
  // You will need both eta and etaK, both of which are Vector3D.

  double cos_theta_i = cos_theta(wi);

  return Vector3D(
    fresnel_term(eta.x, k.x, cos_theta_i),
    fresnel_term(eta.y, k.y, cos_theta_i),
    fresnel_term(eta.z, k.z, cos_theta_i)
  );

}

Vector3D MicrofacetBSDF::f(const Vector3D wo, const Vector3D wi) {
  // TODO: proj3-2, part 3
  // Implement microfacet model here.

  if (wo.z <= 0 || wi.z <= 0) 
  {
    return Vector3D();
  }

  // Calculate the half vector
  Vector3D h = wo + wi;
  h.normalize();
  // n = (0, 0, 1)
  // dot(n, wo) = wo.z
  // dot(n, wi) = wi.z
  Vector3D f = F(wi) * G(wo, wi) * D(h) / (4.0 * wo.z * wi.z);
  return f;
}

Vector3D MicrofacetBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {
  // TODO: proj3-2, part 3
  // *Importance* sample Beckmann normal distribution function (NDF) here.
  // Note: You should fill in the sampled direction *wi and the corresponding *pdf,
  //       and return the sampled BRDF value.

  bool importance = true;

  if (!importance)
  {
    *wi = cosineHemisphereSampler.get_sample(pdf);
    return f(wo, *wi);
  }


  Vector2D r = sampler.get_sample();
  
  double theta_h = atan(sqrt(-(alpha * alpha * log(1.0 - r[0]))));
  double phi_h = 2.0 * PI * r[1];

  double sin_theta_h = sin(theta_h);
  double cos_theta_h = cos(theta_h);
  double tan_theta_h = tan(theta_h);
  double sin_phi_h = sin(phi_h);
  double cos_phi_h = cos(phi_h);

  Vector3D h(sin_theta_h * cos_phi_h, sin_theta_h * sin_phi_h, cos_theta_h);
  h.normalize();

  *wi = -wo + 2.0 * dot(wo, h) * h;

  // Check if wi is valid
  if (wi->z <= 0) 
  {
    *pdf = EPS_F;
    return Vector3D();
  }

  double p_theta_h = exp(-tan_theta_h * tan_theta_h / (alpha * alpha)) * (2.0 * sin_theta_h)
                   / (alpha * alpha * cos_theta_h * cos_theta_h * cos_theta_h);
  double p_phi_h = 0.5 / PI;

  double p_w_h = p_theta_h * p_phi_h / sin_theta_h;
  double p_w_i = p_w_h / (4.0 * dot(*wi, h));

  *pdf = p_w_i;

  return f(wo, *wi);
}

void MicrofacetBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Micofacet BSDF"))
  {
    DragDouble3("eta", &eta[0], 0.005);
    DragDouble3("K", &k[0], 0.005);
    DragDouble("alpha", &alpha, 0.005);
    ImGui::TreePop();
  }
}

// Refraction BSDF //

Vector3D RefractionBSDF::f(const Vector3D wo, const Vector3D wi) {
  return Vector3D();
}

Vector3D RefractionBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {

  // TODO:
  // Implement RefractionBSDF
  
  if (refract(wo, wi, ior))
  {
    *pdf = 1.0;
    double eta;
    // air -> material
    if (wo.z > 0) 
    {
      eta = 1.0 / ior;
    } 
    // material -> air
    else 
    {
      eta = ior;
    }

    return transmittance / abs_cos_theta(*wi) / (eta * eta);
  }
  return Vector3D();
 
}

void RefractionBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Refraction BSDF"))
  {
    DragDouble3("Transmittance", &transmittance[0], 0.005);
    DragDouble("ior", &ior, 0.005);
    ImGui::TreePop();
  }
}

// Glass BSDF //

Vector3D GlassBSDF::f(const Vector3D wo, const Vector3D wi) {
  return Vector3D();
}

Vector3D GlassBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {

  // TODO:
  // Compute Fresnel coefficient and either reflect or refract based on it.

  // compute Fresnel coefficient and use it as the probability of reflection
  // - Fundamentals of Computer Graphics page 305

  // If no refraction
  // total internal reflection
  if (!refract(wo, wi, ior)) 
  {
    reflect(wo, wi);
    *pdf = 1.0;
    return reflectance / abs_cos_theta(*wi);
  }

  // Schlick's approximation
  double r0 = (1.0 - ior) / (1.0 + ior);
  double r1 = r0 * r0;
  double r2 = 1.0 - abs_cos_theta(wo);
  double r3 = r2 * r2;
  double r = r1 + (1.0 - r1) * r3 * r3 * r2;

  if (coin_flip(r)) 
  {
    reflect(wo, wi);
    *pdf = r;
    return r * reflectance / abs_cos_theta(*wi);
  } 
  else 
  {
    *pdf = 1.0 - r;
    double eta;
    // air -> material
    if (wo.z > 0) 
    {
      eta = 1.0 / ior;
    } 
    // material -> air
    else 
    {
      eta = ior;
    }

    return (1.0 - r) * transmittance / abs_cos_theta(*wi) / (eta * eta);
  }
}

void GlassBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Refraction BSDF"))
  {
    DragDouble3("Reflectance", &reflectance[0], 0.005);
    DragDouble3("Transmittance", &transmittance[0], 0.005);
    DragDouble("ior", &ior, 0.005);
    ImGui::TreePop();
  }
}

void BSDF::reflect(const Vector3D wo, Vector3D* wi) {

  // TODO:
  // Implement reflection of wo about normal (0,0,1) and store result in wi.
  
  wi->x = -wo.x;
  wi->y = -wo.y;
  wi->z = wo.z;

}

bool BSDF::refract(const Vector3D wo, Vector3D* wi, double ior) {

  // TODO:
  // Use Snell's Law to refract wo surface and store result ray in wi.
  // Return false if refraction does not occur due to total internal reflection
  // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
  // ray entering the surface through vacuum.

  double eta;
  
  // air -> material
  if (wo.z > 0) 
  {
    eta = 1.0 / ior;
  } 
  // material -> air
  else 
  {
    eta = ior;
  }

  double w_i_z_sqr = 1.0 - eta * eta * (1.0 - wo.z * wo.z);

  if (w_i_z_sqr < 0) 
  {
    return false;
  }

  wi->x = -eta * wo.x;
  wi->y = -eta * wo.y;
  if (wo.z > 0)
  {
    wi->z = -sqrt(w_i_z_sqr);
  }
  else
  {
    wi->z = sqrt(w_i_z_sqr);
  }

  return true;

}

} // namespace CGL
