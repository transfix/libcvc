#ifndef __COLOR_H__
#define __COLOR_H__

CONTOURTILER_BEGIN_NAMESPACE

class Color
{
public:
  Color() {}
  Color(double r, double g, double b) 
    : _r((unsigned char)(r*255.0)), _g((unsigned char)(g*255.0)), _b((unsigned char)(b*255.0)) {}
  ~Color() {}

  double r() const { return _r / 255.0; }
  double g() const { return _g / 255.0; }
  double b() const { return _b / 255.0; }

  bool operator==(const Color& c) const
  { return _r == c._r && _g == c._g && _b == c._b; }

private:  
  unsigned char _r, _g, _b;
};

inline std::size_t hash_value(const Color& color)
{
  std::size_t seed = 0;
  boost::hash_combine(seed, color.r());
  boost::hash_combine(seed, color.g());
  boost::hash_combine(seed, color.b());
  return seed;
}

CONTOURTILER_END_NAMESPACE

#endif
