#include "texture.h"
#include "CGL/color.h"

#include <cmath>
#include <algorithm>

namespace CGL {

  Color Texture::sample(const SampleParams& sp) {
    // TODO: Task 6: Fill this in.
    int level = 0;
    if (sp.lsm == LevelSampleMethod::L_ZERO)
    {
      level = 0;
    }
    else if (sp.lsm == LevelSampleMethod::L_NEAREST)
    {
      float f_level = get_level(sp);
      float left = f_level - std::floor(f_level);
      if (left <= 0.5)
      {
        level = std::floor(f_level);
      }
      else
      {
        level = std::floor(f_level) + 1;
      }
    }
    else if (sp.lsm == LevelSampleMethod::L_LINEAR)
    {
      float f_level = get_level(sp);
      int level1 = std::floor(f_level);
      int level2 = std::floor(f_level) + 1;
      if (sp.psm == PixelSampleMethod::P_NEAREST) 
      {
        return (level2 - f_level) * sample_nearest(sp.p_uv, level1) 
          + (f_level - level1) * sample_nearest(sp.p_uv, level2);
      }
      else if (sp.psm == PixelSampleMethod::P_LINEAR) 
      {
        return (level2 - f_level) * sample_bilinear(sp.p_uv, level1) 
          + (f_level - level1) * sample_bilinear(sp.p_uv, level2);
      }
    }

    if (sp.psm == PixelSampleMethod::P_NEAREST) 
    {
      return sample_nearest(sp.p_uv, level);
    }
    else if (sp.psm == PixelSampleMethod::P_LINEAR) 
    {
      return sample_bilinear(sp.p_uv, level);
    }

    // return magenta for invalid level
    return Color(1, 0, 1);
  }

  float Texture::get_level(const SampleParams& sp) {
    // TODO: Task 6: Fill this in.
    Vector2D dx = sp.p_dx_uv - sp.p_uv;
    dx[0] *= width;
    dx[1] *= height;
    Vector2D dy = sp.p_dy_uv - sp.p_uv;
    dy[0] *= width;
    dy[1] *= height;
    float L = std::max(std::sqrt(dx[0] * dx[0] + dx[1] * dx[1]), std::sqrt(dy[0] * dy[0] + dy[1] * dy[1]));
    float level = std::log2f(L);
    return level;
  }

  Color MipLevel::get_texel(int tx, int ty) {
    return Color(&texels[tx * 3 + ty * width * 3]);
  }

  Color Texture::sample_nearest(Vector2D uv, int level) {
    if (level >= mipmap.size())
    {
      // return magenta for invalid level
      return Color(1, 0, 1);
    }
    // TODO: Task 5: Fill this in.
    auto& mip = mipmap[level];
    size_t mip_width = width / ((int) std::pow(2, level));
    size_t mip_height = height / ((int) std::pow(2, level));
    float u = uv[0];
    float v = uv[1];
    size_t tx, ty;
    float left = u * mip_width - std::floor(u * mip_width);
    if (left <= 0.5)
    {
      tx = std::floor(u * mip_width);
    }
    else
    {
      tx = std::floor(u * mip_width) + 1;
    }
    left = v * mip_height - std::floor(v * mip_height);
    if (left <= 0.5)
    {
      ty = std::floor(v * mip_height);
    }
    else
    {
      ty = std::floor(v * mip_height) + 1;
    }
    return mip.get_texel(tx, ty);
  }

  Color Texture::sample_bilinear(Vector2D uv, int level) {
    if (level >= mipmap.size())
    {
      // return magenta for invalid level
      return Color(1, 0, 1);
    }
    // TODO: Task 5: Fill this in.
    auto& mip = mipmap[level];
    size_t mip_width = width / ((int) std::pow(2, level));
    size_t mip_height = height / ((int) std::pow(2, level));
    float u = uv[0];
    float v = uv[1];
    float left = u * mip_width - std::floor(u * mip_width);
    Color sample_bottom_left = mip.get_texel(std::floor(u * mip_width), std::floor(v * mip_height));
    Color sample_bottom_right = mip.get_texel(std::floor(u * mip_width) + 1, std::floor(v * mip_height));
    Color sample_top_left = mip.get_texel(std::floor(u * mip_width), std::floor(v * mip_height) + 1);
    Color sample_top_right = mip.get_texel(std::floor(u * mip_width) + 1, std::floor(v * mip_height) + 1);
    Color sample_left = sample_bottom_left * (std::floor(v * mip_height) + 1 - v * mip_height) + 
      sample_top_left * (v * mip_height - std::floor(v * mip_height));
    Color sample_right = sample_bottom_right * (std::floor(v * mip_height) + 1 - v * mip_height) + 
      sample_top_right * (v * mip_height - std::floor(v * mip_height));
    Color color = sample_left * (std::floor(u * mip_width) + 1 - u * mip_width) + 
      sample_right * (u * mip_width - std::floor(u * mip_width));
    return color;
  }



  /****************************************************************************/

  // Helpers

  inline void uint8_to_float(float dst[3], unsigned char* src) {
    uint8_t* src_uint8 = (uint8_t*)src;
    dst[0] = src_uint8[0] / 255.f;
    dst[1] = src_uint8[1] / 255.f;
    dst[2] = src_uint8[2] / 255.f;
  }

  inline void float_to_uint8(unsigned char* dst, float src[3]) {
    uint8_t* dst_uint8 = (uint8_t*)dst;
    dst_uint8[0] = (uint8_t)(255.f * max(0.0f, min(1.0f, src[0])));
    dst_uint8[1] = (uint8_t)(255.f * max(0.0f, min(1.0f, src[1])));
    dst_uint8[2] = (uint8_t)(255.f * max(0.0f, min(1.0f, src[2])));
  }

  void Texture::generate_mips(int startLevel) {

    // make sure there's a valid texture
    if (startLevel >= mipmap.size()) {
      std::cerr << "Invalid start level";
    }

    // allocate sublevels
    int baseWidth = mipmap[startLevel].width;
    int baseHeight = mipmap[startLevel].height;
    int numSubLevels = (int)(log2f((float)max(baseWidth, baseHeight)));

    numSubLevels = min(numSubLevels, kMaxMipLevels - startLevel - 1);
    mipmap.resize(startLevel + numSubLevels + 1);

    int width = baseWidth;
    int height = baseHeight;
    for (int i = 1; i <= numSubLevels; i++) {

      MipLevel& level = mipmap[startLevel + i];

      // handle odd size texture by rounding down
      width = max(1, width / 2);
      //assert (width > 0);
      height = max(1, height / 2);
      //assert (height > 0);

      level.width = width;
      level.height = height;
      level.texels = vector<unsigned char>(3 * width * height);
    }

    // create mips
    int subLevels = numSubLevels - (startLevel + 1);
    for (int mipLevel = startLevel + 1; mipLevel < startLevel + subLevels + 1;
      mipLevel++) {

      MipLevel& prevLevel = mipmap[mipLevel - 1];
      MipLevel& currLevel = mipmap[mipLevel];

      int prevLevelPitch = prevLevel.width * 3; // 32 bit RGB
      int currLevelPitch = currLevel.width * 3; // 32 bit RGB

      unsigned char* prevLevelMem;
      unsigned char* currLevelMem;

      currLevelMem = (unsigned char*)&currLevel.texels[0];
      prevLevelMem = (unsigned char*)&prevLevel.texels[0];

      float wDecimal, wNorm, wWeight[3];
      int wSupport;
      float hDecimal, hNorm, hWeight[3];
      int hSupport;

      float result[3];
      float input[3];

      // conditional differentiates no rounding case from round down case
      if (prevLevel.width & 1) {
        wSupport = 3;
        wDecimal = 1.0f / (float)currLevel.width;
      }
      else {
        wSupport = 2;
        wDecimal = 0.0f;
      }

      // conditional differentiates no rounding case from round down case
      if (prevLevel.height & 1) {
        hSupport = 3;
        hDecimal = 1.0f / (float)currLevel.height;
      }
      else {
        hSupport = 2;
        hDecimal = 0.0f;
      }

      wNorm = 1.0f / (2.0f + wDecimal);
      hNorm = 1.0f / (2.0f + hDecimal);

      // case 1: reduction only in horizontal size (vertical size is 1)
      if (currLevel.height == prevLevel.height) {
        //assert (currLevel.height == 1);

        for (int i = 0; i < currLevel.width; i++) {
          wWeight[0] = wNorm * (1.0f - wDecimal * i);
          wWeight[1] = wNorm * 1.0f;
          wWeight[2] = wNorm * wDecimal * (i + 1);

          result[0] = result[1] = result[2] = 0.0f;

          for (int ii = 0; ii < wSupport; ii++) {
            uint8_to_float(input, prevLevelMem + 3 * (2 * i + ii));
            result[0] += wWeight[ii] * input[0];
            result[1] += wWeight[ii] * input[1];
            result[2] += wWeight[ii] * input[2];
          }

          // convert back to format of the texture
          float_to_uint8(currLevelMem + (3 * i), result);
        }

        // case 2: reduction only in vertical size (horizontal size is 1)
      }
      else if (currLevel.width == prevLevel.width) {
        //assert (currLevel.width == 1);

        for (int j = 0; j < currLevel.height; j++) {
          hWeight[0] = hNorm * (1.0f - hDecimal * j);
          hWeight[1] = hNorm;
          hWeight[2] = hNorm * hDecimal * (j + 1);

          result[0] = result[1] = result[2] = 0.0f;
          for (int jj = 0; jj < hSupport; jj++) {
            uint8_to_float(input, prevLevelMem + prevLevelPitch * (2 * j + jj));
            result[0] += hWeight[jj] * input[0];
            result[1] += hWeight[jj] * input[1];
            result[2] += hWeight[jj] * input[2];
          }

          // convert back to format of the texture
          float_to_uint8(currLevelMem + (currLevelPitch * j), result);
        }

        // case 3: reduction in both horizontal and vertical size
      }
      else {

        for (int j = 0; j < currLevel.height; j++) {
          hWeight[0] = hNorm * (1.0f - hDecimal * j);
          hWeight[1] = hNorm;
          hWeight[2] = hNorm * hDecimal * (j + 1);

          for (int i = 0; i < currLevel.width; i++) {
            wWeight[0] = wNorm * (1.0f - wDecimal * i);
            wWeight[1] = wNorm * 1.0f;
            wWeight[2] = wNorm * wDecimal * (i + 1);

            result[0] = result[1] = result[2] = 0.0f;

            // convolve source image with a trapezoidal filter.
            // in the case of no rounding this is just a box filter of width 2.
            // in the general case, the support region is 3x3.
            for (int jj = 0; jj < hSupport; jj++)
              for (int ii = 0; ii < wSupport; ii++) {
                float weight = hWeight[jj] * wWeight[ii];
                uint8_to_float(input, prevLevelMem +
                  prevLevelPitch * (2 * j + jj) +
                  3 * (2 * i + ii));
                result[0] += weight * input[0];
                result[1] += weight * input[1];
                result[2] += weight * input[2];
              }

            // convert back to format of the texture
            float_to_uint8(currLevelMem + currLevelPitch * j + 3 * i, result);
          }
        }
      }
    }
  }

}
