#ifndef __RIGHT_ENUM_H__
#define __RIGHT_ENUM_H__

#include <ContourTiler/config.h>

CONTOURTILER_BEGIN_NAMESPACE

/// Example:
///
/// struct color_def
/// {
///     enum type
///     {
///         red, green, blue
///     };
/// };
/// 
/// typedef Right_enum<color_def> color; // use int as enum holder data type (in common platforms)
/// typedef Right_enum<color_def, unsigned char> color; // use uchar as enum holder
/// 
/// // usage:
/// void f(color p)
/// {
///     p = color::green;
/// }
/// 
/// int main()
/// {
///     color p = color::red;
///     f(p);
/// }

template <typename def, typename inner = typename def::type>
struct Right_enum : def
{
  typedef typename def::type type;
//   typedef typename inner inner;
  inner v;
  Right_enum(type v) : v(static_cast<type>(v)) {}
  operator inner() const {return v;}
};

CONTOURTILER_END_NAMESPACE

#endif
