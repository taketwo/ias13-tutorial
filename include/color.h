#ifndef COLOR_H
#define COLOR_H

#include <ctime>
#include <cmath>
#include <cstdlib>

/// RGBA color.
typedef uint32_t Color;

namespace ColorMap
{

/// Color maps that associate a color to every float from [0..1].
/// This mimics some of the standard MATLAB® color maps, see
/// http://www.mathworks.de/de/help/matlab/ref/colormap.html.
enum Value
{
  /// Ranges from blue to red, and passes through the colors cyan, yellow,
  /// and orange
  JET,
  /// Consists of colors that are shades of green and yellow
  SUMMER,
  /// Varies smoothly from red, through orange, to yellow
  AUTUMN,
  /// Consists of colors that are shades of cyan and magenta
  COOL
};

}

template <typename T> const T&
clamp (const T& value, const T& low, const T& high)
{
  return (value < low ? low : (value > high ? high : value));
}

/** Get a color from R, G, and B values (chars). */
Color
getColorFromRGB (uint8_t r, uint8_t g, uint8_t b)
{
  return static_cast<uint32_t> (r) << 16 |
         static_cast<uint32_t> (g) <<  8 |
         static_cast<uint32_t> (b);
}

/** Get a color from R, G, and B values (floats).
  *
  * The input values are mapped from [0..1] to [0..255]. Input value that are
  * outside of the [0..1] region are clamped automatically. */
Color
getColorFromRGB (float r, float g, float b)
{
  return getColorFromRGB (static_cast<uint8_t> (round (clamp (r, 0.0f, 1.0f) * 255)),
                          static_cast<uint8_t> (round (clamp (g, 0.0f, 1.0f) * 255)),
                          static_cast<uint8_t> (round (clamp (b, 0.0f, 1.0f) * 255)));
}

/** Get R, G, and B values (chars) as array from a color. */
void
getRGBFromColor (Color color, uint8_t* rgb)
{
  rgb[0] = (color >> 16) & 0xFF;
  rgb[1] = (color >>  8) & 0xFF;
  rgb[2] = (color >>  0) & 0xFF;
}

/** Generate a random color. */
Color
generateRandomColor ()
{
  srand (time (0));
  uint8_t r = static_cast<uint8_t> ((rand () % 256));
  uint8_t g = static_cast<uint8_t> ((rand () % 256));
  uint8_t b = static_cast<uint8_t> ((rand () % 256));
  return getColorFromRGB (r, g, b);
}

/** Get the color for a given number in [0..1] using a given colormap.
  *
  * Input values that are outside of the [0..1] region are clamped
  * automatically.
  *
  * \see ColorMap */
Color
getColor (float value, ColorMap::Value colormap = ColorMap::JET)
{
  float v = clamp (value, 0.0f, 1.0f);
  switch (colormap)
  {
    case ColorMap::JET:
      {
        float four_value = v * 4;
        float r = std::min (four_value - 1.5, -four_value + 4.5);
        float g = std::min (four_value - 0.5, -four_value + 3.5);
        float b = std::min (four_value + 0.5, -four_value + 2.5);
        return getColorFromRGB (r, g, b);
      }
    case ColorMap::SUMMER:
      {
        return getColorFromRGB (v, (1.0f + v) * 0.5f, 0.4f);
      }
    case ColorMap::AUTUMN:
      {
        return getColorFromRGB (1.0f, v, 0.0f);
      }
    case ColorMap::COOL:
      {
        return getColorFromRGB (v, 1.0f - v, 1.0f);
      }
  }
  return generateRandomColor ();
}

#endif /* COLOR_H */

