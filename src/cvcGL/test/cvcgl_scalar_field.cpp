/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// cvcgl_scalar_field -- GeometryNode's scalar-field colouring (setScalarField
// and the color_by / colormap / scalar_min / scalar_max state keys). The
// colours it bakes into the polydata's 3-channel uchar array must equal
// cvc::colormap_rgb of the field, survive use_single_color and render-mode
// rebuilds, change in place (same colour array, no cell rebuild) for a range
// or colormap change, and a size mismatch must no-op. Under a texture the
// geometry's colours stay off whatever is recoloured; a field still shows. The
// polydata is reached
// through SceneNode::prop() -> vtkActor -> mapper input. Headless except the
// last section, which renders a field-coloured quad and reads its colour back
// (skipped where nothing rasterises, unless CVC_REQUIRE_RENDER=1). Explicit
// checks + non-zero return (assert() is a no-op under Release).

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/mesh_ops.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/image/image.h>
#include <exception>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkDataArray.h>
#include <vtkMapper.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedCharArray.h>

using cvc::colormap_kind;
using cvc::gl::GeometryNode;
using cvc::gl::GeometryRenderMode;
using cvc::gl::SceneGraph;

namespace {
int g_fail = 0;
void check(bool ok, const char *msg) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", msg);
  if (!ok)
    ++g_fail;
}

const double kNaN = std::numeric_limits<double>::quiet_NaN();
const int kGrid = 4; // a (kGrid+1)^2-vertex grid

// A (k+1) x (k+1) vertex grid over [-5,5]^2 at z = 0, red per-vertex colours,
// functions() = i + j.
cvc::geometry grid(cvc::app &app, int k) {
  cvc::geometry g(app);
  auto &p = g.points();
  auto &col = g.colors();
  auto &f = g.functions();
  for (int j = 0; j <= k; ++j)
    for (int i = 0; i <= k; ++i) {
      p.push_back({-5.0 + 10.0 * i / k, -5.0 + 10.0 * j / k, 0.0});
      col.push_back({1.0, 0.0, 0.0});
      f.push_back(static_cast<double>(i + j));
    }
  auto &t = g.tris();
  for (int j = 0; j < k; ++j)
    for (int i = 0; i < k; ++i) {
      const cvc::geometry::index_t a = j * (k + 1) + i, b = a + 1, c = a + k + 1, d = c + 1;
      t.push_back({a, b, d});
      t.push_back({a, d, c});
    }
  return g;
}

// A closed UV sphere (outward winding) with no colours and no curvatures.
cvc::geometry sphere(cvc::app &app, int slices, int stacks, double r) {
  const double pi = std::acos(-1.0);
  cvc::geometry g(app);
  auto &p = g.points();
  p.push_back({0.0, 0.0, r});
  for (int s = 1; s < stacks; ++s)
    for (int i = 0; i < slices; ++i) {
      const double phi = pi * s / stacks, th = 2.0 * pi * i / slices;
      p.push_back(
          {r * std::sin(phi) * std::cos(th), r * std::sin(phi) * std::sin(th), r * std::cos(phi)});
    }
  p.push_back({0.0, 0.0, -r});
  const cvc::geometry::index_t south = p.size() - 1;
  auto ring = [&](int s, int i) {
    return static_cast<cvc::geometry::index_t>(1 + (s - 1) * slices + (i % slices));
  };
  auto &t = g.tris();
  for (int i = 0; i < slices; ++i) {
    t.push_back({0, ring(1, i), ring(1, i + 1)});
    for (int s = 1; s + 1 < stacks; ++s) {
      t.push_back({ring(s, i), ring(s + 1, i), ring(s + 1, i + 1)});
      t.push_back({ring(s, i), ring(s + 1, i + 1), ring(s, i + 1)});
    }
    t.push_back({south, ring(stacks - 1, i + 1), ring(stacks - 1, i)});
  }
  return g;
}

vtkActor *actorOf(GeometryNode &n) { return vtkActor::SafeDownCast(n.prop()); }

vtkPolyData *polyOf(GeometryNode &n) {
  vtkActor *a = actorOf(n);
  return a ? vtkPolyData::SafeDownCast(a->GetMapper()->GetInput()) : nullptr;
}

vtkUnsignedCharArray *colorArray(GeometryNode &n) {
  vtkPolyData *pd = polyOf(n);
  return pd ? vtkUnsignedCharArray::SafeDownCast(pd->GetPointData()->GetScalars()) : nullptr;
}

// The node's per-vertex colours as packed RGB (empty without a colour array).
std::vector<unsigned char> colorsOf(GeometryNode &n) {
  vtkUnsignedCharArray *c = colorArray(n);
  if (!c || c->GetNumberOfComponents() != 3)
    return {};
  const unsigned char *b = c->GetPointer(0);
  return std::vector<unsigned char>(b, b + 3 * c->GetNumberOfTuples());
}

bool scalarsVisible(GeometryNode &n) {
  vtkActor *a = actorOf(n);
  return a && a->GetMapper()->GetScalarVisibility() != 0;
}

bool sameRange(std::pair<double, double> r, double lo, double hi) {
  return r.first == lo && r.second == hi;
}

bool allRed(const std::vector<unsigned char> &rgb) {
  if (rgb.empty())
    return false;
  for (size_t i = 0; i + 2 < rgb.size(); i += 3)
    if (rgb[i] != 255 || rgb[i + 1] != 0 || rgb[i + 2] != 0)
      return false;
  return true;
}

std::pair<double, double> minMax(const std::vector<double> &v) {
  std::pair<double, double> r(v.front(), v.front());
  for (double x : v) {
    r.first = std::min(r.first, x);
    r.second = std::max(r.second, x);
  }
  return r;
}

std::string stateOf(GeometryNode &n, const char *key) {
  return n.getState(key).value<std::string>();
}

double stateNumber(GeometryNode &n, const char *key) {
  try {
    return std::stod(stateOf(n, key));
  } catch (const std::exception &) {
    return kNaN;
  }
}

void setStateKey(GeometryNode &n, const char *key, const std::string &v) {
  n.getState(key).value(v);
}

void testScalarField(cvc::app &app) {
  std::printf("scalar field on a %dx%d grid\n", kGrid, kGrid);
  SceneGraph sg(app, "sftest");
  const cvc::geometry g = grid(app, kGrid);
  const size_t n = g.num_points();
  auto node = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("grid", g));
  if (!node || !polyOf(*node)) {
    check(false, "the grid is a GeometryNode with a polydata");
    return;
  }

  // Registered state keys and the starting point: geometry colours, no field.
  check(stateOf(*node, "color_by") == "none", "color_by starts as none");
  colormap_kind k0 = colormap_kind::GRAY;
  check(cvc::colormap_from_string(stateOf(*node, "colormap"), k0) && k0 == colormap_kind::VIRIDIS,
        "colormap starts as viridis");
  check(stateOf(*node, "scalar_min").empty() && stateOf(*node, "scalar_max").empty(),
        "scalar_min / scalar_max start empty (auto)");
  check(!node->hasScalarField() && std::isnan(node->scalarRange().first),
        "no scalar field before setScalarField");
  check(allRed(colorsOf(*node)), "geometry colours are shown before any field");

  // The x coordinate, mapped through GRAY over [-5, 5]: known values.
  std::vector<double> x(n);
  for (size_t i = 0; i < n; ++i)
    x[i] = g.points()[i][0];
  node->setScalarField(x, colormap_kind::GRAY, -5.0, 5.0);
  std::vector<unsigned char> rgb = colorsOf(*node);
  check(node->hasScalarField() && sameRange(node->scalarRange(), -5.0, 5.0),
        "setScalarField: hasScalarField, and the range used is the one given");
  check(rgb.size() == 3 * n && scalarsVisible(*node), "one RGB per vertex, scalars visible");
  check(rgb == cvc::colormap_rgb(x, colormap_kind::GRAY, -5.0, 5.0),
        "the colours equal cvc::colormap_rgb(field, GRAY, -5, 5)");
  if (rgb.size() == 3 * n) {
    // vertex 0 is x = -5, vertex kGrid/2 is x = 0, vertex kGrid is x = +5
    const unsigned char *lo = &rgb[0], *mid = &rgb[3 * (kGrid / 2)], *hi = &rgb[3 * kGrid];
    check(lo[0] == lo[1] && lo[1] == lo[2] && mid[0] == mid[1] && mid[1] == mid[2] &&
              hi[0] == hi[1] && hi[1] == hi[2],
          "GRAY colours are grey (r == g == b)");
    check(lo[0] <= 3 && hi[0] >= 252 && mid[0] >= 120 && mid[0] <= 135,
          "GRAY: x = lo is black, x = hi is white, the midpoint mid-grey");
  }
  colormap_kind k1 = colormap_kind::VIRIDIS;
  check(cvc::colormap_from_string(stateOf(*node, "colormap"), k1) && k1 == colormap_kind::GRAY &&
            stateNumber(*node, "scalar_min") == -5.0 && stateNumber(*node, "scalar_max") == 5.0,
        "setScalarField mirrors colormap / scalar_min / scalar_max into the state tree");

  // A range or colormap change recolours IN PLACE: the same colour array, the
  // same points and cells (a rebuild would replace all three).
  vtkPolyData *pd = polyOf(*node);
  vtkSmartPointer<vtkDataArray> colorsBefore = pd->GetPointData()->GetScalars();
  vtkSmartPointer<vtkPoints> pointsBefore = pd->GetPoints();
  vtkSmartPointer<vtkCellArray> polysBefore = pd->GetPolys();
  auto noRebuild = [&]() {
    return pd->GetPointData()->GetScalars() == colorsBefore.Get() &&
           pd->GetPoints() == pointsBefore.Get() && pd->GetPolys() == polysBefore.Get();
  };
  node->setScalarRange(0.0, 5.0);
  check(colorsOf(*node) == cvc::colormap_rgb(x, colormap_kind::GRAY, 0.0, 5.0) &&
            colorsOf(*node) != rgb && sameRange(node->scalarRange(), 0.0, 5.0),
        "setScalarRange recolours with the new range");
  check(noRebuild(), "setScalarRange: same colour array, points and cells (no rebuild)");
  node->setColorMap(colormap_kind::VIRIDIS);
  check(colorsOf(*node) == cvc::colormap_rgb(x, colormap_kind::VIRIDIS, 0.0, 5.0),
        "setColorMap recolours with the new colormap");
  check(noRebuild(), "setColorMap: same colour array, points and cells (no rebuild)");

  // The same through the state keys (what an Ariadne bind / ImGui combo writes).
  setStateKey(*node, "colormap", "magma");
  check(colorsOf(*node) == cvc::colormap_rgb(x, colormap_kind::MAGMA, 0.0, 5.0),
        "colormap state key recolours");
  setStateKey(*node, "scalar_min", "");
  setStateKey(*node, "scalar_max", "");
  check(colorsOf(*node) == cvc::colormap_rgb(x, colormap_kind::MAGMA, -5.0, 5.0) &&
            sameRange(node->scalarRange(), -5.0, 5.0),
        "empty scalar_min / scalar_max = auto: the field's min/max");
  setStateKey(*node, "scalar_max", "2.5");
  check(colorsOf(*node) == cvc::colormap_rgb(x, colormap_kind::MAGMA, -5.0, 2.5) &&
            sameRange(node->scalarRange(), -5.0, 2.5),
        "a numeric scalar_max is used as given");
  setStateKey(*node, "scalar_max", "not-a-number");
  check(sameRange(node->scalarRange(), -5.0, 5.0), "a non-numeric scalar_max is auto");
  const std::vector<unsigned char> magma = colorsOf(*node);
  setStateKey(*node, "colormap", "no-such-map");
  check(colorsOf(*node) == magma, "an unknown colormap name is ignored");
  check(noRebuild(), "state-key recolours: still no rebuild");

  // use_single_color hides the field; switching it off brings the field back.
  node->setUseSingleColor(true);
  check(colorArray(*node) == nullptr && !scalarsVisible(*node),
        "use_single_color on: no per-vertex colours");
  check(node->hasScalarField(), "...but the field is kept");
  node->setUseSingleColor(false);
  check(colorsOf(*node) == magma && scalarsVisible(*node),
        "use_single_color off again: the scalar colours, not the geometry's");

  // Render-mode rebuilds re-apply the field.
  node->setRenderMode(GeometryRenderMode::LINES);
  check(colorsOf(*node) == magma, "LINES rebuild keeps the scalar colours");
  node->setRenderMode(GeometryRenderMode::POINTS);
  check(colorsOf(*node) == magma, "POINTS rebuild keeps the scalar colours");
  node->setRenderMode(GeometryRenderMode::TRIS);
  check(colorsOf(*node) == magma && polyOf(*node)->GetNumberOfPolys() == 2 * kGrid * kGrid,
        "back to TRIS: the cells and the scalar colours");

  // A size mismatch is rejected without touching anything.
  node->setScalarField(std::vector<double>(n - 1, 0.0));
  node->setScalarField(std::vector<double>());
  check(colorsOf(*node) == magma && node->hasScalarField(),
        "a wrong-sized field logs and no-ops (the previous field stays)");

  // clearScalarField: back to color_by (none = the geometry's colours).
  node->clearScalarField();
  check(allRed(colorsOf(*node)) && !node->hasScalarField() && std::isnan(node->scalarRange().first),
        "clearScalarField: the geometry colours again");

  // color_by = function through the state key: geom.functions(), auto min/max.
  const std::vector<double> fn(g.functions().begin(), g.functions().end());
  const auto fr = minMax(fn);
  setStateKey(*node, "color_by", "function");
  check(node->hasScalarField() && sameRange(node->scalarRange(), fr.first, fr.second) &&
            colorsOf(*node) == cvc::colormap_rgb(fn, colormap_kind::MAGMA, fr.first, fr.second),
        "color_by=function colours by geom.functions() over their min/max");

  // An explicit field overrides color_by until it is cleared.
  node->setScalarField(x); // VIRIDIS, auto range
  check(colorsOf(*node) == cvc::colormap_rgb(x, colormap_kind::VIRIDIS, -5.0, 5.0) &&
            stateOf(*node, "color_by") == "function",
        "setScalarField overrides color_by=function");
  node->clearScalarField();
  check(colorsOf(*node) == cvc::colormap_rgb(fn, colormap_kind::VIRIDIS, fr.first, fr.second),
        "clearScalarField falls back to color_by=function");
  setStateKey(*node, "color_by", "bogus");
  check(colorsOf(*node) == cvc::colormap_rgb(fn, colormap_kind::VIRIDIS, fr.first, fr.second),
        "an unknown color_by is ignored");
  setStateKey(*node, "color_by", "colors");
  check(allRed(colorsOf(*node)) && !node->hasScalarField(), "color_by=colors: geom.colors()");

  // A constant field still maps (centred in a unit range), never NaN colours.
  node->setScalarField(std::vector<double>(n, 2.0), colormap_kind::GRAY);
  check(sameRange(node->scalarRange(), 1.5, 2.5) &&
            colorsOf(*node) ==
                cvc::colormap_rgb(std::vector<double>(n, 2.0), colormap_kind::GRAY, 1.5, 2.5),
        "a constant field maps over [v - 0.5, v + 0.5]");
  node->clearScalarField();

  // Curvature kinds from the geometry's own curvatures(): auto = robust range.
  cvc::geometry gc = grid(app, kGrid);
  std::vector<double> k1v(n), k2v(n);
  for (size_t i = 0; i < n; ++i) {
    k1v[i] = g.points()[i][0];
    k2v[i] = 0.25 * g.points()[i][1] - 1.0;
    gc.curvatures().push_back({k1v[i], k2v[i]});
  }
  node->setGeometry(gc);
  node->setColorMap(colormap_kind::TURBO);
  node->setScalarRange(kNaN, kNaN);
  struct Kind {
    const char *name;
    double (*f)(double, double);
  } kinds[] = {{"k1", [](double a, double) { return a; }},
               {"k2", [](double, double b) { return b; }},
               {"mean", [](double a, double b) { return 0.5 * (a + b); }},
               {"gaussian", [](double a, double b) { return a * b; }}};
  for (const Kind &kd : kinds) {
    std::vector<double> v(n);
    for (size_t i = 0; i < n; ++i)
      v[i] = kd.f(k1v[i], k2v[i]);
    const auto rr = cvc::robust_range(v);
    setStateKey(*node, "color_by", kd.name);
    const std::string what = std::string("color_by=") + kd.name +
                             " colours by curvatures() over the robust (percentile) range";
    check(node->hasScalarField() && sameRange(node->scalarRange(), rr.first, rr.second) &&
              colorsOf(*node) == cvc::colormap_rgb(v, colormap_kind::TURBO, rr.first, rr.second),
          what.c_str());
  }

  // No curvatures on the geometry: computed once with libigl, else no field.
  const cvc::geometry s = sphere(app, 16, 10, 2.0);
  node->setGeometry(s); // color_by is still "gaussian"; switch to mean
  setStateKey(*node, "color_by", "mean");
  if (cvc::mesh_ops_available()) {
    check(node->hasScalarField() && colorsOf(*node).size() == 3 * s.num_points() &&
              std::isfinite(node->scalarRange().first) && std::isfinite(node->scalarRange().second),
          "color_by=mean on a mesh without curvatures computes them (libigl)");
  } else {
    check(!node->hasScalarField() && colorArray(*node) == nullptr,
          "color_by=mean without curvatures or libigl: no field, the single colour");
  }

  // A setGeometry with a different point count drops an explicit field.
  setStateKey(*node, "color_by", "none");
  node->setScalarField(std::vector<double>(s.num_points(), 1.0));
  check(node->hasScalarField(), "a field on the sphere");
  node->setGeometry(g);
  check(!node->hasScalarField() && allRed(colorsOf(*node)),
        "setGeometry with another point count drops the explicit field");
}

// A texture supplies the surface colour: geom.colors() must not tint it, and no
// scalar-colouring change (colormap, range, color_by) or rebuild may switch them
// back on. A scalar field is an explicit request and does show over it.
// clearTexture brings the geometry's colours back -- through the TCoords rebuild
// (zero-copy texture) and through the recolour (copied texture) alike.
void testTexturedColours(cvc::app &app) {
  std::printf("vertex colours under a texture\n");
  SceneGraph sg(app, "sftex");
  cvc::geometry g = grid(app, kGrid);
  const size_t n = g.num_points();
  for (size_t i = 0; i < n; ++i) {
    const auto &p = g.const_points()[i];
    g.uvs().push_back({(p[0] + 5.0) / 10.0, (p[1] + 5.0) / 10.0});
  }
  auto node = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("tex", g));
  if (!node || !polyOf(*node)) {
    check(false, "the textured grid is a GeometryNode with a polydata");
    return;
  }
  node->setUseSingleColor(false);
  check(allRed(colorsOf(*node)) && scalarsVisible(*node),
        "the geometry colours show before a texture");

  const cvc::image tex(4, 4, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
  node->setTexture(tex); // RGBA8: the zero-copy path (V flipped in the TCoords)
  check(actorOf(*node)->GetTexture() != nullptr && !scalarsVisible(*node),
        "setTexture: the texture, not the vertex colours");
  setStateKey(*node, "colormap", "magma");
  check(!scalarsVisible(*node),
        "a colormap state change (color_by=none) leaves the vertex colours off under the texture");
  node->setColorMap(colormap_kind::GRAY);
  node->setScalarRange(0.0, 1.0);
  check(!scalarsVisible(*node), "setColorMap / setScalarRange: still off");
  setStateKey(*node, "scalar_min", "");
  setStateKey(*node, "scalar_max", "");
  setStateKey(*node, "color_by", "colors");
  check(!scalarsVisible(*node), "scalar_min / scalar_max / color_by=colors: still off");
  node->setUseSingleColor(true);
  node->setUseSingleColor(false);
  node->setRenderMode(GeometryRenderMode::LINES);
  node->setRenderMode(GeometryRenderMode::TRIS);
  check(!scalarsVisible(*node), "use_single_color and render-mode rebuilds: still off");

  std::vector<double> x(n);
  for (size_t i = 0; i < n; ++i)
    x[i] = g.const_points()[i][0];
  node->setScalarField(x, colormap_kind::GRAY, -5.0, 5.0);
  check(scalarsVisible(*node) &&
            colorsOf(*node) == cvc::colormap_rgb(x, colormap_kind::GRAY, -5.0, 5.0),
        "an explicit scalar field shows over the texture");
  node->setColorMap(colormap_kind::VIRIDIS);
  check(scalarsVisible(*node), "...and stays shown through a colormap change");
  node->setTexture(tex);
  check(scalarsVisible(*node), "setTexture over a shown field keeps it shown");
  node->clearScalarField();
  check(!scalarsVisible(*node) && allRed(colorsOf(*node)),
        "clearScalarField: the vertex colours again, hidden under the texture");

  node->clearTexture();
  check(actorOf(*node)->GetTexture() == nullptr && scalarsVisible(*node) && allRed(colorsOf(*node)),
        "clearTexture (zero-copy texture): the geometry colours show again");
  node->setTexture(tex, /*zeroCopy=*/false);
  check(!scalarsVisible(*node), "setTexture (copied): the vertex colours off");
  node->clearTexture();
  check(actorOf(*node)->GetTexture() == nullptr && scalarsVisible(*node) && allRed(colorsOf(*node)),
        "clearTexture (copied texture): the geometry colours show again");
}

// Render a quad coloured by a constant field with a flat material and read the
// centre pixel back: the colour must be the colormap's, which a float array
// routed through VTK's lookup table would not give.
void testRenderedColour(cvc::app &app) {
  std::printf("rendered colour\n");
  try {
    SceneGraph sg(app, "sfgl");
    sg.setDiagnosticChromeVisible(false);
    const cvc::geometry g = grid(app, 2);
    auto node = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("quad", g));
    if (!node) {
      check(false, "the quad is a GeometryNode");
      return;
    }
    node->setAmbient(1.0); // unlit: the colour reads back exactly
    node->setDiffuse(0.0);
    node->setSpecular(0.0);
    node->setColor(1.0, 1.0, 1.0);
    node->setUseSingleColor(true);

    cvc::gl::SceneRenderer sr(sg, 64, 64, /*offscreen=*/true);
    sr.setBackground(0, 0, 0);
    sr.setCamera(0, 0, 30, 0, 0, 0, 0, 1, 0, 40, 1, 100);
    auto centre = [&]() {
      const std::vector<unsigned char> px = sr.frameRGB(); // renders
      const int w = sr.frameWidth(), h = sr.frameHeight();
      const size_t at = 3 * (static_cast<size_t>(h / 2) * w + w / 2);
      return px.size() >= at + 3 ? std::vector<unsigned char>(px.begin() + at, px.begin() + at + 3)
                                 : std::vector<unsigned char>();
    };

    // Control: the plain white quad must rasterise.
    const std::vector<unsigned char> white = centre();
    if (white.size() != 3 || white[0] < 200 || white[1] < 200 || white[2] < 200) {
      const char *require = std::getenv("CVC_REQUIRE_RENDER");
      if (require && *require && std::string(require) != "0")
        check(false, "CVC_REQUIRE_RENDER is set, but the white control quad did not rasterise");
      else
        std::printf("  SKIP: this build did not rasterise (the white control quad drew nothing)\n");
      sr.close();
      return;
    }

    node->setUseSingleColor(false);
    node->setScalarField(std::vector<double>(g.num_points(), 0.8), colormap_kind::VIRIDIS, 0.0,
                         1.0);
    const std::vector<unsigned char> want =
        cvc::colormap_rgb(std::vector<double>(1, 0.8), colormap_kind::VIRIDIS, 0.0, 1.0);
    const std::vector<unsigned char> got = centre();
    bool near = got.size() == 3 && want.size() == 3;
    for (size_t c = 0; near && c < 3; ++c)
      near = std::abs(int(got[c]) - int(want[c])) <= 8;
    char msg[160];
    std::snprintf(msg, sizeof msg, "rendered colour (%d,%d,%d) is the colormap's (%d,%d,%d)",
                  got.size() == 3 ? got[0] : -1, got.size() == 3 ? got[1] : -1,
                  got.size() == 3 ? got[2] : -1, want.size() == 3 ? want[0] : -1,
                  want.size() == 3 ? want[1] : -1, want.size() == 3 ? want[2] : -1);
    check(near, msg);
    sr.close();
  } catch (const std::exception &e) {
    std::printf("  SKIP: no offscreen GL context (%s)\n", e.what());
  } catch (...) {
    std::printf("  SKIP: no offscreen GL context\n");
  }
}
} // namespace

int main() {
  cvc::app app;
  testScalarField(app);
  testTexturedColours(app);
  testRenderedColour(app);
  std::printf("%s: cvcgl_scalar_field (%d check%s failed)\n", g_fail == 0 ? "PASS" : "FAIL", g_fail,
              g_fail == 1 ? "" : "s");
  return g_fail == 0 ? 0 : 1;
}
