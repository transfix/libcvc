#ifndef __TILER_OPTIONS__
#define __TILER_OPTIONS__

#include <string>
#include <ContourTiler/common.h>

CONTOURTILER_BEGIN_NAMESPACE

class Tiler_options
{
public:
  Tiler_options() :
    _base_name(""),
    _output_dir("."),
    _z_scale(1),
    _output_intermediate_tiles(false),
    _output_phases_tiles(false),
    _output_bottom(false),
    _output_top(false),
    _output_otv_pairs(false),
    _output_otv(false),
    _output_vertex_labels(false),
    _output_untiled_regions_gnuplot(false),
    _output_intermediate_raw(false),
    _output_intermediate_gnuplot(false),
    _output_phases_raw(false),
    _output_phases_gnuplot(false),
    _output_raw(false),
    _output_gnuplot(false),
    _multi(false),
    _remove_intersections(false),
    _collinear_epsilon(0),
    _contour_curation_delta(0.00000001),
    _color_r(-1),
    _color_g(-1),
    _color_b(-1),
    _color(false),
    _interp_untiled_regions(true),
    _correspondence_overlap(-1)
  {}

  std::string& base_name() { return _base_name; }
  const std::string& base_name() const { return _base_name; }
  std::string& output_dir() { return _output_dir; }
  const std::string& output_dir() const { return _output_dir; }
  double& z_scale() { return _z_scale; }
  const double& z_scale() const { return _z_scale; }
  bool& output_bottom() { return _output_bottom; }
  bool output_bottom() const { return _output_bottom; }
  bool& output_top() { return _output_top; }
  bool output_top() const { return _output_top; }
  bool& output_otv_pairs() { return _output_otv_pairs; }
  bool output_otv_pairs() const { return _output_otv_pairs; }
  bool& output_otv() { return _output_otv; }
  bool output_otv() const { return _output_otv; }
  bool& output_vertex_labels() { return _output_vertex_labels; }
  bool output_vertex_labels() const { return _output_vertex_labels; }
  bool& output_untiled_regions_gnuplot() { return _output_untiled_regions_gnuplot; }
  bool output_untiled_regions_gnuplot() const { return _output_untiled_regions_gnuplot; }
  bool& output_intermediate_raw() { return _output_intermediate_raw; }
  bool output_intermediate_raw() const { return _output_intermediate_raw; }
  bool& output_intermediate_gnuplot() { return _output_intermediate_gnuplot; }
  bool output_intermediate_gnuplot() const { return _output_intermediate_gnuplot; }
  bool& output_phases_raw() { return _output_phases_raw; }
  bool output_phases_raw() const { return _output_phases_raw; }
  bool& output_phases_gnuplot() { return _output_phases_gnuplot; }
  bool output_phases_gnuplot() const { return _output_phases_gnuplot; }
  bool& output_raw() { return _output_raw; }
  bool output_raw() const { return _output_raw; }
  bool& output_gnuplot() { return _output_gnuplot; }
  bool output_gnuplot() const { return _output_gnuplot; }
  bool& multi() { return _multi; }
  bool multi() const { return _multi; }
  bool& remove_intersections() { return _remove_intersections; }
  bool remove_intersections() const { return _remove_intersections; }
  Number_type& collinear_epsilon() { return _collinear_epsilon; }
  Number_type collinear_epsilon() const { return _collinear_epsilon; }
  Number_type& contour_curation_delta() { return _contour_curation_delta; }
  Number_type contour_curation_delta() const { return _contour_curation_delta; }
  int& color_r() { return _color_r; }
  int color_r() const { return _color_r; }
  int& color_g() { return _color_g; }
  int color_g() const { return _color_g; }
  int& color_b() { return _color_b; }
  int color_b() const { return _color_b; }
  bool& color() { return _color; }
  bool color() const { return _color; }
  bool& interp_untiled_regions() { return _interp_untiled_regions; }
  bool interp_untiled_regions() const { return _interp_untiled_regions; }
  Number_type& correspondence_overlap() { return _correspondence_overlap; }
  Number_type correspondence_overlap() const { return _correspondence_overlap; }

private:
  std::string _base_name;
  std::string _output_dir;
  double _z_scale;
  bool _output_intermediate_tiles;
  bool _output_phases_tiles;
  bool _output_bottom;
  bool _output_top;
  bool _output_otv_pairs;
  bool _output_otv;
  bool _output_vertex_labels;
  bool _output_untiled_regions_gnuplot;
  bool _output_intermediate_raw;
  bool _output_intermediate_gnuplot;
  bool _output_phases_raw;
  bool _output_phases_gnuplot;
  bool _output_raw;
  bool _output_gnuplot;
  bool _multi;
  bool _remove_intersections;
  Number_type _collinear_epsilon;
  Number_type _contour_curation_delta;
  int _color_r;
  int _color_g;
  int _color_b;
  bool _color;
  bool _interp_untiled_regions;
  Number_type _correspondence_overlap;
};

CONTOURTILER_END_NAMESPACE

#endif
