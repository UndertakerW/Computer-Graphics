#include "rasterizer.h"
#include "texture.h"

using namespace std;

namespace CGL
{

  RasterizerImp::RasterizerImp(PixelSampleMethod psm, LevelSampleMethod lsm, AntiAliasingMethod aam,
                               size_t width, size_t height,
                               unsigned int sample_rate)
  {
    this->psm = psm;
    this->lsm = lsm;
    this->aam = aam;
    this->width = width;
    this->height = height;
    this->sample_rate = sample_rate;

    int num_subpixel = std::floor(std::sqrt(sample_rate));
    sample_buffer.resize(width * num_subpixel * height * num_subpixel, Color::White);
    subpixel_count_buffer.resize(width * height, 0);
    fxaa_threshold = 0.05;
    fxaa_rate = 0.8;
  }

  // Used by rasterize_point and rasterize_line
  void RasterizerImp::fill_pixel(size_t x, size_t y, Color c)
  {
    // TODO: Task 2: You might need to this function to fix points and lines (such as the black rectangle border in test4.svg)
    // NOTE: You are not required to implement proper supersampling for points and lines
    // It is sufficient to use the same color for all supersamples of a pixel for points and lines (not triangles)
    int num_subpixel = 1;
    if (aam == AntiAliasingMethod::AA_SSAA)
    {
      num_subpixel = std::floor(std::sqrt(sample_rate));
    }
    else if (aam == AntiAliasingMethod::AA_MSAA || aam == AntiAliasingMethod::AA_FXAA)
    {
      num_subpixel = 1;
    }
    sample_buffer[y * width * num_subpixel + x] = c;
  }

  // Rasterize a point: simple example to help you start familiarizing
  // yourself with the starter code.
  //
  void RasterizerImp::rasterize_point(float x, float y, Color color)
  {
    // fill in the nearest pixel
    int sx = (int)floor(x);
    int sy = (int)floor(y);

    // check bounds
    if (sx < 0 || sx >= width)
      return;
    if (sy < 0 || sy >= height)
      return;

    int num_subpixel = 1;
    if (aam == AntiAliasingMethod::AA_SSAA)
    {
      num_subpixel = std::floor(std::sqrt(sample_rate));
    }
    else if (aam == AntiAliasingMethod::AA_MSAA || aam == AntiAliasingMethod::AA_FXAA)
    {
      num_subpixel = 1;
    }
    for (int u = 0; u < num_subpixel; ++u)
    {
      for (int v = 0; v < num_subpixel; ++v)
      {
        fill_pixel(sx * num_subpixel + u, sy * num_subpixel + v, color);
      }
    }
    // fill_pixel(sx, sy, color);
    return;
  }

  // Rasterize a line.
  void RasterizerImp::rasterize_line(float x0, float y0,
                                     float x1, float y1,
                                     Color color)
  {
    if (x0 > x1)
    {
      swap(x0, x1);
      swap(y0, y1);
    }

    float pt[] = {x0, y0};
    float m = (y1 - y0) / (x1 - x0);
    float dpt[] = {1, m};
    int steep = abs(m) > 1;
    if (steep)
    {
      dpt[0] = x1 == x0 ? 0 : 1 / abs(m);
      dpt[1] = x1 == x0 ? (y1 - y0) / abs(y1 - y0) : m / abs(m);
    }

    while (floor(pt[0]) <= floor(x1) && abs(pt[1] - y0) <= abs(y1 - y0))
    {
      rasterize_point(pt[0], pt[1], color);
      pt[0] += dpt[0];
      pt[1] += dpt[1];
    }
  }

  static bool insideTriangle(double x, double y,
                             float x0, float y0,
                             float x1, float y1,
                             float x2, float y2)
  {

    Eigen::Vector2f AB, BC, CA;
    AB << x1 - x0, y1 - y0;
    BC << x2 - x1, y2 - y1;
    CA << x0 - x2, y0 - y2;

    Eigen::Vector2f AP, BP, CP;
    AP << x - x0, y - y0;
    BP << x - x1, y - y1;
    CP << x - x2, y - y2;

    // Check the cross products
    // If all positive or all negative
    float AB_AP = AB[0] * AP[1] - AB[1] * AP[0];
    float BC_BP = BC[0] * BP[1] - BC[1] * BP[0];
    float CA_CP = CA[0] * CP[1] - CA[1] * CP[0];

    bool result = (AB_AP >= 0 && BC_BP >= 0 && CA_CP >= 0) || (AB_AP <= 0 && BC_BP <= 0 && CA_CP <= 0);

    return result;
  }

  std::tuple<float, float, float> RasterizerImp::computeBarycentric2D(float x, float y,
                                                                      float x0, float y0,
                                                                      float x1, float y1,
                                                                      float x2, float y2)
  {
    float c1 = (x * (y1 - y2) + (x2 - x1) * y + x1 * y2 - x2 * y1) / (x0 * (y1 - y2) + (x2 - x1) * y0 + x1 * y2 - x2 * y1);
    float c2 = (x * (y2 - y0) + (x0 - x2) * y + x2 * y0 - x0 * y2) / (x1 * (y2 - y0) + (x0 - x2) * y1 + x2 * y0 - x0 * y2);
    float c3 = (x * (y0 - y1) + (x1 - x0) * y + x0 * y1 - x1 * y0) / (x2 * (y0 - y1) + (x1 - x0) * y2 + x0 * y1 - x1 * y0);
    return {c1, c2, c3};
  }

  void RasterizerImp::set_pixel(size_t x, size_t y, const Color &color)
  {
    auto ind = (height - 1 - y) * width + x;
    rgb_framebuffer_target[ind] = color.r * 255;
    rgb_framebuffer_target[ind + 1] = color.g * 255;
    rgb_framebuffer_target[ind + 2] = color.b * 255;
  }

  // Rasterize a triangle.
  void RasterizerImp::rasterize_triangle(float x0, float y0,
                                         float x1, float y1,
                                         float x2, float y2,
                                         Color color)
  {
    // TODO: Task 1: Implement basic triangle rasterization here, no supersampling
    // TODO: Task 2: Update to implement super-sampled rasterization

    // Calculate the boundary box
    float x_min = std::min(x0, std::min(x1, x2));
    float x_max = std::max(x0, std::max(x1, x2));
    float y_min = std::min(y0, std::min(y1, y2));
    float y_max = std::max(y0, std::max(y1, y2));

    // Refine the boundary box using ceil() and floor() functions
    // Since the edges of the triangle should be also resterized
    x_min = std::floor(x_min);
    x_max = std::ceil(x_max);
    y_min = std::floor(y_min);
    y_max = std::ceil(y_max);

    int num_subpixel = std::floor(std::sqrt(sample_rate));
    float subpixel_size = 1.0 / num_subpixel;
    float subpixel_weight = 1.0 / (num_subpixel * num_subpixel);

    // For each pixel inside the boundary box
    for (int x = x_min; x <= x_max; x++)
    {
      for (int y = y_min; y <= y_max; y++)
      {
        // Check whether the center of the (supersampled) pixel is inside the triangle
        if (aam == AntiAliasingMethod::AA_SSAA)
        {
          for (int u = 0; u < num_subpixel; u++)
          {
            for (int v = 0; v < num_subpixel; v++)
            {
              float x_sample = (float)x + subpixel_size * u + subpixel_size / 2;
              float y_sample = (float)y + subpixel_size * v + subpixel_size / 2;
              if (insideTriangle(x_sample, y_sample, x0, y0, x1, y1, x2, y2))
              {
                fill_pixel(x * num_subpixel + u, y * num_subpixel + v, color);
              }
            }
          }
        }
        else if (aam == AntiAliasingMethod::AA_MSAA)
        {
          int points_inside_triangle = 0;
          for (int i = 0; i < num_subpixel; i++)
          {
            for (int j = 0; j < num_subpixel; j++)
            {
              float x_sample = (float)x + subpixel_size * i + subpixel_size / 2;
              float y_sample = (float)y + subpixel_size * j + subpixel_size / 2;
              if (insideTriangle(x_sample, y_sample, x0, y0, x1, y1, x2, y2))
              {
                points_inside_triangle++;
              }
            }
          }
          if (points_inside_triangle > 0)
          {
            Color msaa_color = color * points_inside_triangle * subpixel_weight;
            if (subpixel_count_buffer[y * width + x] > 0)
            {
              if (subpixel_count_buffer[y * width + x] + points_inside_triangle <= num_subpixel * num_subpixel)
              {
                Color bg_color = sample_buffer[y * width + x];
                msaa_color += bg_color;
                subpixel_count_buffer[y * width + x] += points_inside_triangle;
              }
              else
              {
                Color bg_color = sample_buffer[y * width + x] * (num_subpixel * num_subpixel - points_inside_triangle)
                  * (1.0 / subpixel_count_buffer[y * width + x]);
                msaa_color += bg_color;
                subpixel_count_buffer[y * width + x] = num_subpixel * num_subpixel;
              }
            }
            else
            {
              subpixel_count_buffer[y * width + x] += points_inside_triangle;
            }
            fill_pixel(x, y, msaa_color);
          }
        }
        else if (aam == AntiAliasingMethod::AA_FXAA)
        {
          float x_sample = (float)x + 0.5;
          float y_sample = (float)y + 0.5;
          if (insideTriangle(x_sample, y_sample, x0, y0, x1, y1, x2, y2))
          {
            fill_pixel(x, y, color);
          }
        }
      }
    }
  }

  void RasterizerImp::rasterize_interpolated_color_triangle(float x0, float y0, Color c0,
                                                            float x1, float y1, Color c1,
                                                            float x2, float y2, Color c2)
  {
    // TODO: Task 4: Rasterize the triangle, calculating barycentric coordinates and using them to interpolate vertex colors across the triangle
    // Hint: You can reuse code from rasterize_triangle
    // Calculate the boundary box
    float x_min = std::min(x0, std::min(x1, x2));
    float x_max = std::max(x0, std::max(x1, x2));
    float y_min = std::min(y0, std::min(y1, y2));
    float y_max = std::max(y0, std::max(y1, y2));

    // Refine the boundary box using ceil() and floor() functions
    // Since the edges of the triangle should be also resterized
    x_min = std::floor(x_min);
    x_max = std::ceil(x_max);
    y_min = std::floor(y_min);
    y_max = std::ceil(y_max);

    int num_subpixel = std::floor(std::sqrt(sample_rate));
    float subpixel_size = 1.0 / num_subpixel;
    float subpixel_weight = 1.0 / (num_subpixel * num_subpixel);

    // For each pixel inside the boundary box
    for (int x = x_min; x <= x_max; x++)
    {
      for (int y = y_min; y <= y_max; y++)
      {
        if (aam == AntiAliasingMethod::AA_SSAA)
        {
          // Check whether the center of the (supersampled) pixel is inside the triangle
          for (int u = 0; u < num_subpixel; u++)
          {
            for (int v = 0; v < num_subpixel; v++)
            {
              float x_sample = (float)x + subpixel_size * u + subpixel_size / 2;
              float y_sample = (float)y + subpixel_size * v + subpixel_size / 2;
              if (insideTriangle(x_sample, y_sample, x0, y0, x1, y1, x2, y2))
              {
                std::tuple<float, float, float> barycentric = computeBarycentric2D(x_sample, y_sample,
                                                                                   x0, y0, x1, y1, x2, y2);
                fill_pixel(x * num_subpixel + u, y * num_subpixel + v,
                           std::get<0>(barycentric) * c0 +
                               std::get<1>(barycentric) * c1 +
                               std::get<2>(barycentric) * c2);
              }
            }
          }
        }
        else if (aam == AntiAliasingMethod::AA_MSAA)
        {
          int points_inside_triangle = 0;
          Color msaa_color = Color::Black;
          // Check whether the center of the (supersampled) pixel is inside the triangle
          for (int u = 0; u < num_subpixel; u++)
          {
            for (int v = 0; v < num_subpixel; v++)
            {
              float x_sample = (float)x + subpixel_size * u + subpixel_size / 2;
              float y_sample = (float)y + subpixel_size * v + subpixel_size / 2;
              if (insideTriangle(x_sample, y_sample, x0, y0, x1, y1, x2, y2))
              {
                points_inside_triangle++;
                std::tuple<float, float, float> barycentric = computeBarycentric2D(x_sample, y_sample,
                                                                                   x0, y0, x1, y1, x2, y2);
                Color sampled_color = std::get<0>(barycentric) * c0 + 
                  std::get<1>(barycentric) * c1 + 
                  std::get<2>(barycentric) * c2;
                msaa_color += sampled_color * subpixel_weight;
              }
            }
          }
          if (points_inside_triangle > 0)
          {
            if (subpixel_count_buffer[y * width + x] > 0)
            {
              if (subpixel_count_buffer[y * width + x] + points_inside_triangle <= num_subpixel * num_subpixel)
              {
                Color bg_color = sample_buffer[y * width + x];
                msaa_color += bg_color;
                subpixel_count_buffer[y * width + x] += points_inside_triangle;
              }
              else
              {
                Color bg_color = sample_buffer[y * width + x] * (num_subpixel * num_subpixel - points_inside_triangle)
                  * (1.0 / subpixel_count_buffer[y * width + x]);
                msaa_color += bg_color;
                subpixel_count_buffer[y * width + x] = num_subpixel * num_subpixel;
              }
            }
            else
            {
              subpixel_count_buffer[y * width + x] += points_inside_triangle;
            }
            fill_pixel(x, y, msaa_color);
          }
        }
        else if (aam == AntiAliasingMethod::AA_FXAA)
        {
          float x_sample = (float)x + 0.5;
          float y_sample = (float)y + 0.5;
          if (insideTriangle(x_sample, y_sample, x0, y0, x1, y1, x2, y2))
          {
            std::tuple<float, float, float> barycentric = computeBarycentric2D(x_sample, y_sample,
                                                                                x0, y0, x1, y1, x2, y2);
            fill_pixel(x, y,
                        std::get<0>(barycentric) * c0 +
                            std::get<1>(barycentric) * c1 +
                            std::get<2>(barycentric) * c2);
          }
        }
      }
    }
  }

  void RasterizerImp::rasterize_textured_triangle(float x0, float y0, float u0, float v0,
                                                  float x1, float y1, float u1, float v1,
                                                  float x2, float y2, float u2, float v2,
                                                  Texture &tex)
  {
    // TODO: Task 5: Fill in the SampleParams struct and pass it to the tex.sample function.
    // TODO: Task 6: Set the correct barycentric differentials in the SampleParams struct.
    // Hint: You can reuse code from rasterize_triangle/rasterize_interpolated_color_triangle
    float x_min = std::min(x0, std::min(x1, x2));
    float x_max = std::max(x0, std::max(x1, x2));
    float y_min = std::min(y0, std::min(y1, y2));
    float y_max = std::max(y0, std::max(y1, y2));

    // Refine the boundary box using ceil() and floor() functions
    // Since the edges of the triangle should be also resterized
    x_min = std::floor(x_min);
    x_max = std::ceil(x_max);
    y_min = std::floor(y_min);
    y_max = std::ceil(y_max);

    int num_subpixel = std::floor(std::sqrt(sample_rate));
    float subpixel_size = 1.0 / num_subpixel;
    float subpixel_weight = 1.0 / (num_subpixel * num_subpixel);

    // For each pixel inside the boundary box
    for (int x = x_min; x <= x_max; x++)
    {
      for (int y = y_min; y <= y_max; y++)
      {
        if (aam == AntiAliasingMethod::AA_SSAA)
        {
          for (int u = 0; u < num_subpixel; u++)
          {
            for (int v = 0; v < num_subpixel; v++)
            {
              float x_sample = (float)x + subpixel_size * u + subpixel_size / 2;
              float y_sample = (float)y + subpixel_size * v + subpixel_size / 2;
              if (insideTriangle(x_sample, y_sample, x0, y0, x1, y1, x2, y2))
              {
                std::tuple<float, float, float> barycentric = computeBarycentric2D(x_sample, y_sample,
                                                                                  x0, y0, x1, y1, x2, y2);
                float tex_u = std::get<0>(barycentric) * u0 + std::get<1>(barycentric) * u1 + std::get<2>(barycentric) * u2;
                float tex_v = std::get<0>(barycentric) * v0 + std::get<1>(barycentric) * v1 + std::get<2>(barycentric) * v2;
                struct SampleParams sampleParams;
                sampleParams.p_uv[0] = tex_u;
                sampleParams.p_uv[1] = tex_v;
                std::tuple<float, float, float> barycentric_px = computeBarycentric2D(x_sample + subpixel_size, y_sample,
                                                                                      x0, y0, x1, y1, x2, y2);
                float tex_ux = std::get<0>(barycentric_px) * u0 + std::get<1>(barycentric_px) * u1 + std::get<2>(barycentric_px) * u2;
                float tex_vx = std::get<0>(barycentric_px) * v0 + std::get<1>(barycentric_px) * v1 + std::get<2>(barycentric_px) * v2;
                sampleParams.p_dx_uv[0] = tex_ux;
                sampleParams.p_dx_uv[1] = tex_vx;
                std::tuple<float, float, float> barycentric_py = computeBarycentric2D(x_sample, y_sample + subpixel_size,
                                                                                      x0, y0, x1, y1, x2, y2);
                float tex_uy = std::get<0>(barycentric_py) * u0 + std::get<1>(barycentric_py) * u1 + std::get<2>(barycentric_py) * u2;
                float tex_vy = std::get<0>(barycentric_py) * v0 + std::get<1>(barycentric_py) * v1 + std::get<2>(barycentric_py) * v2;
                sampleParams.p_dy_uv[0] = tex_uy;
                sampleParams.p_dy_uv[1] = tex_vy;
                sampleParams.psm = psm;
                sampleParams.lsm = lsm;

                Color color = tex.sample(sampleParams);
                fill_pixel(x * num_subpixel + u, y * num_subpixel + v, color);
              }
            }
          }
        }
        else if (aam == AntiAliasingMethod::AA_MSAA)
        {
          int points_inside_triangle = 0;
          Color msaa_color = Color::Black;
          for (int u = 0; u < num_subpixel; u++)
          {
            for (int v = 0; v < num_subpixel; v++)
            {
              float x_sample = (float)x + subpixel_size * u + subpixel_size / 2;
              float y_sample = (float)y + subpixel_size * v + subpixel_size / 2;
              if (insideTriangle(x_sample, y_sample, x0, y0, x1, y1, x2, y2))
              {
                points_inside_triangle++;
                std::tuple<float, float, float> barycentric = computeBarycentric2D(x_sample, y_sample,
                                                                                  x0, y0, x1, y1, x2, y2);
                float tex_u = std::get<0>(barycentric) * u0 + std::get<1>(barycentric) * u1 + std::get<2>(barycentric) * u2;
                float tex_v = std::get<0>(barycentric) * v0 + std::get<1>(barycentric) * v1 + std::get<2>(barycentric) * v2;
                struct SampleParams sampleParams;
                sampleParams.p_uv[0] = tex_u;
                sampleParams.p_uv[1] = tex_v;
                std::tuple<float, float, float> barycentric_px = computeBarycentric2D(x_sample + subpixel_size, y_sample,
                                                                                      x0, y0, x1, y1, x2, y2);
                float tex_ux = std::get<0>(barycentric_px) * u0 + std::get<1>(barycentric_px) * u1 + std::get<2>(barycentric_px) * u2;
                float tex_vx = std::get<0>(barycentric_px) * v0 + std::get<1>(barycentric_px) * v1 + std::get<2>(barycentric_px) * v2;
                sampleParams.p_dx_uv[0] = tex_ux;
                sampleParams.p_dx_uv[1] = tex_vx;
                std::tuple<float, float, float> barycentric_py = computeBarycentric2D(x_sample, y_sample + subpixel_size,
                                                                                      x0, y0, x1, y1, x2, y2);
                float tex_uy = std::get<0>(barycentric_py) * u0 + std::get<1>(barycentric_py) * u1 + std::get<2>(barycentric_py) * u2;
                float tex_vy = std::get<0>(barycentric_py) * v0 + std::get<1>(barycentric_py) * v1 + std::get<2>(barycentric_py) * v2;
                sampleParams.p_dy_uv[0] = tex_uy;
                sampleParams.p_dy_uv[1] = tex_vy;
                sampleParams.psm = psm;
                sampleParams.lsm = lsm;

                Color sampled_color = tex.sample(sampleParams);
                msaa_color += sampled_color * subpixel_weight;
              }
            }
          }
          if (points_inside_triangle > 0)
          {
            if (subpixel_count_buffer[y * width + x] > 0)
            {
              if (subpixel_count_buffer[y * width + x] + points_inside_triangle <= num_subpixel * num_subpixel)
              {
                Color bg_color = sample_buffer[y * width + x];
                msaa_color += bg_color;
                subpixel_count_buffer[y * width + x] += points_inside_triangle;
              }
              else
              {
                Color bg_color = sample_buffer[y * width + x] * (num_subpixel * num_subpixel - points_inside_triangle)
                  * (1.0 / subpixel_count_buffer[y * width + x]);
                msaa_color += bg_color;
                subpixel_count_buffer[y * width + x] = num_subpixel * num_subpixel;
              }
            }
            else
            {
              subpixel_count_buffer[y * width + x] += points_inside_triangle;
            }
            fill_pixel(x, y, msaa_color);
          }
        }
        else if (aam == AntiAliasingMethod::AA_FXAA)
        {
          float x_sample = (float)x + 0.5;
          float y_sample = (float)y + 0.5;
          if (insideTriangle(x_sample, y_sample, x0, y0, x1, y1, x2, y2))
          {
            std::tuple<float, float, float> barycentric = computeBarycentric2D(x_sample, y_sample,
                                                                              x0, y0, x1, y1, x2, y2);
            float tex_u = std::get<0>(barycentric) * u0 + std::get<1>(barycentric) * u1 + std::get<2>(barycentric) * u2;
            float tex_v = std::get<0>(barycentric) * v0 + std::get<1>(barycentric) * v1 + std::get<2>(barycentric) * v2;
            struct SampleParams sampleParams;
            sampleParams.p_uv[0] = tex_u;
            sampleParams.p_uv[1] = tex_v;
            std::tuple<float, float, float> barycentric_px = computeBarycentric2D(x_sample + subpixel_size, y_sample,
                                                                                  x0, y0, x1, y1, x2, y2);
            float tex_ux = std::get<0>(barycentric_px) * u0 + std::get<1>(barycentric_px) * u1 + std::get<2>(barycentric_px) * u2;
            float tex_vx = std::get<0>(barycentric_px) * v0 + std::get<1>(barycentric_px) * v1 + std::get<2>(barycentric_px) * v2;
            sampleParams.p_dx_uv[0] = tex_ux;
            sampleParams.p_dx_uv[1] = tex_vx;
            std::tuple<float, float, float> barycentric_py = computeBarycentric2D(x_sample, y_sample + subpixel_size,
                                                                                  x0, y0, x1, y1, x2, y2);
            float tex_uy = std::get<0>(barycentric_py) * u0 + std::get<1>(barycentric_py) * u1 + std::get<2>(barycentric_py) * u2;
            float tex_vy = std::get<0>(barycentric_py) * v0 + std::get<1>(barycentric_py) * v1 + std::get<2>(barycentric_py) * v2;
            sampleParams.p_dy_uv[0] = tex_uy;
            sampleParams.p_dy_uv[1] = tex_vy;
            sampleParams.psm = psm;
            sampleParams.lsm = lsm;

            Color color = tex.sample(sampleParams);
            fill_pixel(x, y, color);
          }
        }
      }
    }
  }

  void RasterizerImp::set_sample_rate(unsigned int rate)
  {
    // TODO: Task 2: You may want to update this function for supersampling support

    this->sample_rate = rate;
    if (aam == AntiAliasingMethod::AA_SSAA)
    {
      int num_subpixel = std::floor(std::sqrt(sample_rate));
      this->sample_buffer.resize(width * num_subpixel * height * num_subpixel, Color::White);
    }
    else if (aam == AntiAliasingMethod::AA_MSAA || aam == AntiAliasingMethod::AA_FXAA)
    {
      this->sample_buffer.resize(width * height, Color::White);
    }
  }

  void RasterizerImp::set_framebuffer_target(unsigned char *rgb_framebuffer,
                                             size_t width, size_t height)
  {
    // TODO: Task 2: You may want to update this function for supersampling support

    this->width = width;
    this->height = height;
    this->rgb_framebuffer_target = rgb_framebuffer;

    int num_subpixel = 1;
    if (aam == AntiAliasingMethod::AA_SSAA)
    {
      num_subpixel = std::floor(std::sqrt(sample_rate));
    }
    else if (aam == AntiAliasingMethod::AA_MSAA || aam == AntiAliasingMethod::AA_FXAA)
    {
      num_subpixel = 1;
    }
    this->sample_buffer.resize(width * num_subpixel * height * num_subpixel, Color::White);
    subpixel_count_buffer.resize(width * height, 0);
  }

  void RasterizerImp::clear_buffers()
  {
    std::fill(rgb_framebuffer_target, rgb_framebuffer_target + 3 * width * height, 255);
    std::fill(sample_buffer.begin(), sample_buffer.end(), Color::White);
    std::fill(subpixel_count_buffer.begin(), subpixel_count_buffer.end(), 0);
  }

  // This function is called at the end of rasterizing all elements of the
  // SVG file.  If you use a supersample buffer to rasterize SVG elements
  // for antialising, you could use this call to fill the target framebuffer
  // pixels from the supersample buffer data.
  //
  void RasterizerImp::resolve_to_framebuffer()
  {
    // TODO: Task 2: You will likely want to update this function for supersampling support

    int num_subpixel = 1;
    if (aam == AntiAliasingMethod::AA_SSAA)
    {
      num_subpixel = std::floor(std::sqrt(sample_rate));
    }
    else if (aam == AntiAliasingMethod::AA_MSAA || aam == AntiAliasingMethod::AA_FXAA)
    {
      num_subpixel = 1;
    }

    float subpixel_size = 1.0 / num_subpixel;
    float subpixel_weight = 1.0 / (num_subpixel * num_subpixel);

    for (int x = 0; x < width; ++x)
    {
      for (int y = 0; y < height; ++y)
      {
        Color col = Color::Black;
        for (int u = 0; u < num_subpixel; ++u)
        {
          for (int v = 0; v < num_subpixel; ++v)
          {
            col += subpixel_weight * sample_buffer[(y * num_subpixel + v) * (width * num_subpixel) + x * num_subpixel + u];
          }
        }
        if (aam == AntiAliasingMethod::AA_MSAA)
        {
          col = sample_buffer[y * width + x];
          int num_subpixels = std::floor(std::sqrt(sample_rate)) * std::floor(std::sqrt(sample_rate));
          if (subpixel_count_buffer[y * width + x] > num_subpixels)
          {
            std::cout << subpixel_count_buffer[y * width + x] << endl;
            cout << col.r << " " << col.g << " " << col.b << endl;
          }
          if (subpixel_count_buffer[y * width + x] > 0 && subpixel_count_buffer[y * width + x] < num_subpixels)
          {
            Color bg_color = Color::White;
            col += bg_color * (num_subpixels - subpixel_count_buffer[y * width + x]) * (1.0 / num_subpixels);
          }
        }
        else if (aam == AntiAliasingMethod::AA_FXAA)
        {
          col = sample_buffer[y * width + x];
          if (0 < x && x < width && 0 < y && y < height)
          {
            Color& col_upper_left = sample_buffer[(y + 1) * width + (x - 1)];
            Color& col_left = sample_buffer[y * width + (x - 1)];
            Color& col_lower_left = sample_buffer[(y - 1) * width + (x - 1)];
            Color& col_upper = sample_buffer[(y + 1) * width + x];
            Color& col_lower = sample_buffer[(y - 1) * width + x];
            Color& col_upper_right = sample_buffer[(y + 1) * width + (x + 1)];
            Color& col_right = sample_buffer[y * width + (x + 1)];
            Color& col_lower_right = sample_buffer[(y - 1) * width + (x + 1)];

            Color col_diff_ul_lr = col_upper_left - col_lower_right;
            Color col_diff_l_r = col_left - col_right;
            Color col_diff_ll_ur = col_lower_left - col_upper_right;
            Color col_diff_l_u = col_lower - col_upper;

            Color col_avg_ul_lr = col_upper_left * fxaa_rate + col_lower_right * fxaa_rate + col * (1.0 - 2 * fxaa_rate);
            Color col_avg_l_r = col_left * fxaa_rate + col_right * fxaa_rate + col * (1.0 - 2 * fxaa_rate);
            Color col_avg_ll_ur = col_lower_left * fxaa_rate + col_upper_right * fxaa_rate + col * (1.0 - 2 * fxaa_rate);
            Color col_avg_l_u = col_lower * fxaa_rate + col_upper * fxaa_rate + col * (1.0 - 2 * fxaa_rate);

            std::vector<Color> col_diffs = {col_diff_ul_lr, col_diff_l_r, col_diff_ll_ur, col_diff_l_u};
            std::vector<Color> col_avgs = {col_avg_ul_lr, col_avg_l_r, col_avg_ll_ur, col_avg_l_u};

            int max_diff_index = 0;
            int max_g_diff = 0;
            int max_b_diff = 0;
            int mix_count = 0;
            Color fxaa_color = Color::Black;
            for (int i = 0; i < 4; ++i)
            {
              if (col_diffs[i].r * col_diffs[i].r 
                + col_diffs[i].g * col_diffs[i].g
                + col_diffs[i].b * col_diffs[i].b 
                > col_diffs[max_diff_index].r * col_diffs[max_diff_index].r
                + col_diffs[max_diff_index].g * col_diffs[max_diff_index].b
                + col_diffs[max_diff_index].b * col_diffs[max_diff_index].b)
              {
                max_diff_index = i;
              }
              if (col_diffs[i].r * col_diffs[i].r 
                + col_diffs[i].g * col_diffs[i].g
                + col_diffs[i].b * col_diffs[i].b 
                > fxaa_threshold)
              {
                mix_count++;
                fxaa_color += col_avgs[i];
              }
            }
            if (mix_count > 0)
            {
              fxaa_color = col * (1.0 - fxaa_rate) + (col_upper_left + col_left + col_lower_left + col_upper + col_lower
              + col_upper_right + col_right + col_lower_right) * (1.0 / 8.0) * fxaa_rate;
              col = fxaa_color;
            }
          }
        }
        for (int k = 0; k < 3; ++k)
        {
          this->rgb_framebuffer_target[3 * (y * width + x) + k] = (&col.r)[k] * 255;
        }
      }
    }
  }

  Rasterizer::~Rasterizer() {}

} // CGL
