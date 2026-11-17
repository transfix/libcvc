#include <ContourTiler/Z_adjustments.h>

#define FIXED_EPSILON 0

CONTOURTILER_BEGIN_NAMESPACE

//--------------------------------------
// is_legal
//
/// Returns true if bz is closer to a's 
/// z home than az
//--------------------------------------
template <typename Tile_handle>
bool Z_adjustments<Tile_handle>::is_legal(const Point_3& py, const Point_3& pg, const Point_3& qy,
					  Number_type y_z_home, Number_type g_z_home)
{
  Number_type az = py.z();
  Number_type bz = pg.z();

#ifdef FIXED_EPSILON
  if (y_z_home > g_z_home) {
    return az > bz + _epsilon;
  }
  return az < bz - _epsilon;
#else
  Point_3 A(pg.x()-qy.x(), pg.y()-qy.y(), pg.z()-qy.z());
  Point_3 B(py.x()-qy.x(), py.y()-qy.y(), py.z()-qy.z());
  Number_type d = _epsilon;
  int dir = sgn(g_z_home - pg.z());
  Number_type e = epsilon(A, B, d, dir);
  if (y_z_home > g_z_home)
    return e >= 0;
  return e <= 0;
#endif
}

/// dir: 1 if epsilon is expected to be positive, -1 if otherwise.
template <typename Tile_handle>
Number_type Z_adjustments<Tile_handle>::epsilon(const Point_3& A, const Point_3& B, Number_type delta, int dir) const {
#ifdef FIXED_EPSILON
  return _epsilon;
#else
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("Z_adjustments.epsilon");
  LOG4CPLUS_TRACE(logger, "A = " << pp(A) << " B = " << pp(B) << " delta = " << delta);

  Number_type d = delta;
  Number_type a = pow(A.y() + B.y(), 2) + pow(A.x() + B.x(), 2) - d*d;

  // The portion that is commented-out is the equation in the paper.  There was an
  // error in the paper, however, and that has been corrected in the second equation.
  //     Number_type b = 2 * ((A.x() + B.x()) * (A.z()*B.x() - A.x()*B.z()) - 
  // 			 (A.y() + B.y())*(A.y()*B.z()-A.z()*B.y()) - d*d*A.z());
  Number_type b = 2 * ((A.x() + B.x()) * (A.z()*B.x() - A.x()*B.z()) - 
		       (A.y() + B.y())*(A.y()*B.z()-A.z()*B.y()) - d*d*B.z());

  Number_type c = pow(A.y()*B.z()-A.z()*B.y(), 2) + pow(A.z()*B.x()-A.x()*B.z(), 2) +
    pow(A.x()*B.y()-A.y()*B.x(), 2) - d*d*(B.x()*B.x() + B.y()*B.y() + B.z()*B.z());

  Number_type r1, r2, c1, c2;
  solve_quad(a, b, c, r1, r2, c1, c2);
  if (!solve_quad(a, b, c, r1, r2, c1, c2)) {
    stringstream ss;
    ss << "Expected a solution to the quadratic.  ";
    ss << "a = " << a << " b = " << b << " c = " << c;
    throw logic_error(ss.str());
  }
  if (sgn(r1) == dir) {
    if (sgn(r2) == dir) {
      return (dir*r1 > dir*r2 ? r1 : r2);
    }
    return r1;
  }
  return r2;
#endif
}

/// d - delta
template <typename Tile_handle>
Number_type Z_adjustments<Tile_handle>::epsilon(const Point_3& A, const Point_3& B, Number_type d, int dir, Number_type sg) const {
  return sg * epsilon(A, B, d, dir);
}

template <typename Tile_handle>
Number_type Z_adjustments<Tile_handle>::epsilon() const
{ return _epsilon; }

template <typename Tile_handle>
void Z_adjustments<Tile_handle>::add(const Point_3& py, const Point_3& pg, int yi, int gi, const Point_3& qy,
				     Number_type y_z_home, Number_type g_z_home)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("Z_adjustments.add");

  if (y_z_home == g_z_home) {
    LOG4CPLUS_ERROR(logger, " y_z_home = " << y_z_home << " g_z_home = " << g_z_home);
    throw logic_error("y_z_home == g_z_home");
  }

  LOG4CPLUS_TRACE(logger, "py = " << pp(py) << " pg = " << pp(pg) << 
		  " yi = " << yi << " gi = " << gi << " qy = " << pp(qy));

  Number_type zmid = (pg.z() + py.z()) / (Number_type) 2.0; // halfway between two z values
  Number_type gsign = (g_z_home > y_z_home) ? 1 : -1;


#ifdef FIXED_EPSILON
  Number_type gz = zmid + gsign * (_epsilon/2.0);
  Number_type yz = zmid - gsign * (_epsilon/2.0);

  if (py.z() == y_z_home) {
    yz = y_z_home;
    gz = y_z_home + gsign * _epsilon;
  }
  else if (pg.z() == g_z_home) {
    gz = g_z_home;
    yz = g_z_home - gsign * _epsilon;
  }
  else {
    const Number_type gdiff = (gz > g_z_home) ? 1 : -1;
    const Number_type ydiff = (yz > y_z_home) ? 1 : -1;
    if (gdiff == gsign) {
      gz = zmid;
      yz = zmid - gsign * _epsilon;
    }
    else if (ydiff != gsign) {
      yz = zmid;
      gz = zmid + gsign * _epsilon;
    }

    const Number_type gdiff2 = (gz > g_z_home) ? 1 : -1;
    const Number_type ydiff2 = (yz > y_z_home) ? 1 : -1;
    if (gdiff2 == gsign) {
      throw logic_error("overshot green");
    }
    if (ydiff2 != gsign) {
      LOG4CPLUS_ERROR(logger, "_epsilon = " << _epsilon << " yz = " << yz << " gz = " << gz << " y_z_home = " << y_z_home << " g_z_home = " << g_z_home);
      throw logic_error("overshot yellow");
    }
  }

#else
  // Calculate epsilon
  Point_3 A(pg.x()-qy.x(), pg.y()-qy.y(), pg.z()-qy.z());
  Point_3 B(py.x()-qy.x(), py.y()-qy.y(), py.z()-qy.z());

  Number_type d = _epsilon;
  int dir = sgn(g_z_home - pg.z());
  Number_type e = epsilon(A, B, d, dir);

  Number_type gz = pg.z() + gsign * 2 * e;
  Number_type yz = py.z() - gsign * 2 * e;

  if (py.z() == y_z_home) {
    yz = y_z_home;
    gz = y_z_home + gsign * 4 * e;
  }
  else if (pg.z() == g_z_home) {
    gz = g_z_home;
    yz = g_z_home - gsign * 4 * e;
  }
  else {
    Number_type shift = 0;

    const Number_type gdiff = (gz > g_z_home) ? 1 : -1;
    const Number_type ydiff = (yz > y_z_home) ? 1 : -1;
    if (gdiff == gsign) {
      Number_type req = fabs(gz-g_z_home);
      shift = -(req + min(fabs(g_z_home-zmid)/2.0, (fabs(yz-y_z_home)-req)/2.0)) * gsign;
    }
    else if (ydiff != gsign) {
      Number_type req = fabs(yz-y_z_home);
      shift = req + min(fabs(y_z_home-zmid)/2.0, (fabs(gz-g_z_home)-req)/2.0) * gsign;
    }
    gz += shift;
    yz += shift;

    const Number_type gdiff2 = (gz > g_z_home) ? 1 : -1;
    const Number_type ydiff2 = (yz > y_z_home) ? 1 : -1;
    if (gdiff2 == gsign) {
      throw logic_error("overshot green");
    }
    if (ydiff2 != gsign) {
      LOG4CPLUS_ERROR(logger, "_epsilon = " << _epsilon << " yz = " << yz << " gz = " << gz << " y_z_home = " << y_z_home << " g_z_home = " << g_z_home);
      throw logic_error("overshot yellow");
    }
  }

//   if (pg.z() == y_z_home || pg.z() == g_z_home)
//     gz = pg.z();
//   else if (fabs(pg.z() - g_z_home) < fabs(e))
//     gz = pg.z();

//   if (py.z() == y_z_home || py.z() == g_z_home)
//     yz = py.z();
//   else if (fabs(py.z() - y_z_home) < fabs(e))
//     yz = py.z();
#endif

// #ifdef FIXED_EPSILON
//   Number_type gz = zmid + gsign * (_epsilon/2.0);
//   Number_type yz = zmid - gsign * (_epsilon/2.0);

//   if (py.z() == y_z_home) {
//     yz = y_z_home;
//     gz = py.z() + gsign * _epsilon;
//   }
//   else if (pg.z() == g_z_home) {
//     gz = g_z_home;
//     yz = pg.z() - gsign * _epsilon;
//   }
//   else {
//     const Number_type gdiff = (gz > g_z_home) ? 1 : -1;
//     const Number_type ydiff = (yz > y_z_home) ? 1 : -1;
//     if (gdiff == gsign) {
//       gz = zmid;
//       yz = zmid - gsign * _epsilon;
//     }
//     else if (ydiff != gsign) {
//       yz = zmid;
//       gz = zmid + gsign * _epsilon;
//     }

//     const Number_type gdiff2 = (gz > g_z_home) ? 1 : -1;
//     const Number_type ydiff2 = (yz > y_z_home) ? 1 : -1;
//     if (gdiff2 == gsign) {
//       throw logic_error("overshot green");
//     }
//     if (ydiff2 != gsign) {
//       LOG4CPLUS_ERROR(logger, "_epsilon = " << _epsilon << " yz = " << yz << " gz = " << gz << " y_z_home = " << y_z_home << " g_z_home = " << g_z_home);
//       throw logic_error("overshot yellow");
//     }
//   }

// #else
//   // Calculate epsilon
//   Point_3 A(pg.x()-qy.x(), pg.y()-qy.y(), pg.z()-qy.z());
//   Point_3 B(py.x()-qy.x(), py.y()-qy.y(), py.z()-qy.z());

//   Number_type d = _epsilon;
//   int dir = sgn(g_z_home - pg.z());
//   Number_type e = epsilon(A, B, d, dir);

//   Number_type gz = pg.z() + gsign * 2 * e;
//   if (pg.z() == y_z_home || pg.z() == g_z_home)
//     gz = pg.z();
//   else if (fabs(pg.z() - g_z_home) < fabs(e))
//     gz = pg.z();

//   Number_type yz = py.z() - gsign * 2 * e;
//   if (py.z() == y_z_home || py.z() == g_z_home)
//     yz = py.z();
//   else if (fabs(py.z() - y_z_home) < fabs(e))
//     yz = py.z();
// #endif

  Point_3 pg_bar(pg.x(), pg.y(), gz);
  Point_3 py_bar(py.x(), py.y(), yz);

  if (!is_legal(py, pg, qy, y_z_home, g_z_home)) {
//     LOG4CPLUS_TRACE(logger, "Not legal.  _epsilon = " << _epsilon << " epsilon = " << e << 
// 		    " old pg = " << pp(pg) << " new pg = " << pg_bar);
    _adjustments[gi][pg] = pg_bar;
    _adjustments[yi][py] = py_bar;
  }
  else {
    _adjustments[gi][pg] = pg;
    _adjustments[yi][py] = py;
  }
}

template class Z_adjustments<boost::shared_ptr<Triangle> >;

CONTOURTILER_END_NAMESPACE
