// GeometryNode::setPBR and SceneGraph's image-based lighting (docs/ARIADNE.md §5.4, "Physically
// based materials", builds a node's `material: { pbr: }` and the scene's `environment:` on them).
//
// A 2 x 2 plate faces +z under a camera straight above it, lit by a light straight above, so its
// centre pixel has N = V = L = H and VTK's BRDF has a closed form there (vtkPBRFunctions.glsl).
// With roughness 1: D = 1/pi, V = 1/4, F = F0 = 0.04 (a dielectric of IOR 1.5). For a light of
// intensity 1, the lights' sum times pi is 0.96 * albedo + 0.01, and the pixel is that to the
// power 1/2.2.
//
// Pins:
//   A. state and property: setPBR switches the actor to VTK_PBR and back; metallic, roughness,
//      coat and emissive reach the property (emissive made linear), clamped to 0..1; a value set
//      before the switch applies; setPBR(true) sets diffuse to 1;
//   B. a lit plate of colour (0.8, 0.2, 0.2) draws (202, 58, 58): the base colour made linear and
//      the light times pi. VTK as it stands draws (120, 34, 34), and linear with no pi, pale;
//   C. emissive (0.5, 0.25, 0) on a black plate, unlit, draws (128, 64, 0);
//   D. setPBR(false) draws it Phong-shaded again;
//   E. image-based lighting: off, an unlit plate is black; on, a grey environment lights it; the
//      sky lights a face looking up and the ground one looking down; a polished metal shows the
//      environment's colour; a new environment shows at once;
//   F. the environment follows the scene to a new renderer, and leaves the old one.
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <memory>
#include <string>
#include <vector>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>

using cvc::gl::GeometryNode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;

static int fails = 0;
static void chk(bool ok, const std::string &what) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok)
    ++fails;
}

namespace {
const int kSize = 64;

std::shared_ptr<GeometryNode> addPlate(SceneGraph &sg) {
  cvc::geometry g;
  const double xs[4] = {-1, 1, 1, -1}, ys[4] = {-1, -1, 1, 1};
  for (int i = 0; i < 4; ++i) {
    cvc::geometry::point_t p;
    p[0] = xs[i];
    p[1] = ys[i];
    p[2] = 0.0;
    g.points().push_back(p);
  }
  const int idx[2][3] = {{0, 1, 2}, {0, 2, 3}}; // counter-clockwise from +z: it faces up
  for (const auto &tri : idx) {
    cvc::geometry::tri_t t;
    t[0] = tri[0];
    t[1] = tri[1];
    t[2] = tri[2];
    g.tris().push_back(t);
  }
  return std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("plate", g));
}

struct Rgb {
  int r = 0, g = 0, b = 0;
};

// The frame's centre pixel, looking at the plate from straight above (or below).
Rgb centre(SceneRenderer &r, bool fromBelow = false) {
  r.setCamera(0, 0, fromBelow ? -10 : 10, 0, 0, 0, 0, 1, 0, 30.0, 1.0, 100.0);
  const std::vector<unsigned char> f = r.frameRGB();
  const std::size_t i = (static_cast<std::size_t>(kSize / 2) * kSize + kSize / 2) * 3;
  return {f[i], f[i + 1], f[i + 2]};
}

std::string fmt(const Rgb &c) {
  char buf[48];
  std::snprintf(buf, sizeof(buf), "(%d, %d, %d)", c.r, c.g, c.b);
  return buf;
}

bool near(const Rgb &c, int r, int g, int b, int tol) {
  return std::abs(c.r - r) <= tol && std::abs(c.g - g) <= tol && std::abs(c.b - b) <= tol;
}

// The pixel the closed form above gives, for one channel of the base colour.
int litByOne(double srgb) {
  const double lit = 0.96 * std::pow(srgb, 2.2) + 0.04 * (1.0 / 3.14159265) * 0.25 * 3.14159265;
  return static_cast<int>(std::lround(255.0 * std::pow(lit, 1.0 / 2.2)));
}
} // namespace

int main() {
  cvc::app app;
  SceneGraph sg(app);
  sg.setDiagnosticChromeVisible(false);
  const int sun = sg.addDirectionalLight(0.0, 90.0, 1.0, 1.0, 1.0, 1.0); // straight above
  std::shared_ptr<GeometryNode> plate = addPlate(sg);
  if (!plate) {
    std::printf("FAIL  the plate is not a GeometryNode\n");
    return 1;
  }
  auto *actor = vtkActor::SafeDownCast(plate->prop());
  vtkProperty *prop = actor ? actor->GetProperty() : nullptr;
  if (!prop) {
    std::printf("FAIL  the plate has no actor\n");
    return 1;
  }
  SceneRenderer r(sg, kSize, kSize, /*offscreen=*/true);
  r.setBackground(0.0, 0.0, 0.0);
  sg.processEvents();

  // Before any PBR: Phong-shaded, diffuse 1 and no specular, face on to the light, the plate shows
  // its colour (what B's claim of "as bright as Phong" is measured against, and D's baseline).
  plate->setColor(0.8, 0.2, 0.2);
  plate->setDiffuse(1.0);
  plate->setSpecular(0.0);
  sg.processEvents();
  const Rgb phong0 = centre(r);
  std::printf("A. state and property\n");
  chk(near(phong0, 204, 51, 51, 3), "Phong, diffuse 1, face on: the colour " + fmt(phong0));
  const int phong = prop->GetInterpolation();
  plate->setDiffuse(0.7);
  plate->setMetallic(0.25); // before the switch
  plate->setColor(0.8, 0.2, 0.2);
  plate->setPBR(true);
  sg.processEvents();
  chk(plate->pbr() && prop->GetInterpolation() == VTK_PBR, "setPBR(true): the property is PBR");
  chk(std::fabs(prop->GetMetallic() - 0.25) < 1e-12, "a metallic set before the switch applies");
  chk(std::fabs(prop->GetDiffuse() - 1.0) < 1e-12, "setPBR(true) sets diffuse to 1");
  plate->setRoughness(1.7);
  plate->setMetallic(-2.0);
  plate->setCoat(0.6, 0.3);
  plate->setEmissive(0.5, 0.25, 0.0);
  sg.processEvents();
  chk(prop->GetRoughness() == 1.0 && prop->GetMetallic() == 0.0, "roughness and metallic clamp");
  chk(std::fabs(prop->GetCoatStrength() - 0.6) < 1e-12 &&
          std::fabs(prop->GetCoatRoughness() - 0.3) < 1e-12,
      "the coat's strength and roughness");
  const double *e = prop->GetEmissiveFactor();
  chk(std::fabs(e[0] - std::pow(0.5, 2.2)) < 1e-9 && std::fabs(e[1] - std::pow(0.25, 2.2)) < 1e-9 &&
          e[2] == 0.0,
      "emissive made linear: " + std::to_string(e[0]) + ", " + std::to_string(e[1]));
  chk(plate->getState("pbr").value<bool>(), "the pbr key is in state");
  plate->getState("pbr").value(std::string("off")); // as a script writes it
  chk(!plate->pbr() && prop->GetInterpolation() == phong, "pbr written off in state: Phong");
  plate->getState("pbr").value(std::string("True"));
  chk(plate->pbr() && prop->GetInterpolation() == VTK_PBR, "and True: PBR again");
  plate->getState("roughness").value(0.75); // a write to the key, as a script's
  chk(std::fabs(prop->GetRoughness() - 0.75) < 1e-12, "a roughness written to state applies");
  plate->setCoat(0.0, 0.0);
  plate->setEmissive(0.0, 0.0, 0.0);
  plate->setMetallic(0.0);
  plate->setRoughness(1.0);
  sg.processEvents();

  std::printf("B. a lit plate's colour\n");
  const Rgb lit = centre(r);
  const int want_r = litByOne(0.8), want_g = litByOne(0.2);
  chk(near(lit, want_r, want_g, want_g, 5),
      "(0.8, 0.2, 0.2) lit by 1 draws " + fmt(lit) + ", want (" + std::to_string(want_r) + ", " +
          std::to_string(want_g) + ", " + std::to_string(want_g) + ")");
  chk(lit.r > 160 && lit.g < 90, "not VTK's (120, 34, 34) nor a pale (227, 123, 123)");

  std::printf("C. emissive\n");
  sg.setLightIntensity(sun, 0.0);
  plate->setColor(0.0, 0.0, 0.0);
  plate->setEmissive(0.5, 0.25, 0.0);
  sg.processEvents();
  const Rgb glow = centre(r);
  chk(near(glow, 128, 64, 0, 4), "emissive (0.5, 0.25, 0), unlit, draws " + fmt(glow));
  plate->setEmissive(0.0, 0.0, 0.0);
  sg.processEvents();
  chk(near(centre(r), 0, 0, 0, 2), "and with none, black");

  std::printf("D. back to Phong\n");
  sg.setLightIntensity(sun, 1.0);
  plate->setColor(0.8, 0.2, 0.2);
  plate->setPBR(false);
  sg.processEvents();
  const Rgb ph = centre(r);
  chk(!plate->pbr() && prop->GetInterpolation() == phong, "setPBR(false) restores interpolation");
  chk(near(ph, phong0.r, phong0.g, phong0.b, 2),
      "Phong again, as before PBR: " + fmt(ph) + ", was " + fmt(phong0));

  std::printf("E. image-based lighting\n");
  plate->setPBR(true);
  plate->setColor(1.0, 1.0, 1.0);
  sg.setLightIntensity(sun, 0.0);
  sg.processEvents();
  chk(!sg.imageBasedLighting() && !r.renderer()->GetUseImageBasedLighting(), "off by default");
  chk(near(centre(r), 0, 0, 0, 2), "unlit and no environment: black");
  const double grey[3] = {0.5, 0.5, 0.5};
  sg.setEnvironment(grey, grey, grey);
  sg.setImageBasedLighting(true);
  sg.processEvents();
  const Rgb amb = centre(r);
  chk(r.renderer()->GetUseImageBasedLighting() && sg.environmentTexture() &&
          r.renderer()->GetEnvironmentTexture() == sg.environmentTexture(),
      "on: the renderer lights with the scene's environment");
  // 0.96 x 0.5^2.2 from the irradiance, plus a little reflected: about 125..135.
  chk(amb.r > 110 && amb.r < 155 && std::abs(amb.r - amb.g) <= 3 && std::abs(amb.r - amb.b) <= 3,
      "a grey environment lights a white plate grey: " + fmt(amb));

  const double white[3] = {1, 1, 1}, mid[3] = {0.5, 0.5, 0.5}, black[3] = {0, 0, 0};
  sg.setEnvironment(white, mid, black);
  sg.processEvents();
  const Rgb up = centre(r), down = centre(r, /*fromBelow=*/true);
  chk(up.r > down.r + 60, "the sky lights the face looking up " + fmt(up) +
                              ", the ground the one looking down " + fmt(down));

  const double blue[3] = {0.2, 0.4, 0.8};
  sg.setEnvironment(blue, blue, blue);
  plate->setMetallic(1.0);
  plate->setRoughness(0.2);
  sg.processEvents();
  const Rgb metal = centre(r);
  chk(metal.b > metal.g + 20 && metal.g > metal.r + 20 && metal.b > 120,
      "a polished metal shows the environment's blue: " + fmt(metal));
  double sky[3], horizon[3], ground[3];
  sg.environment(sky, horizon, ground);
  chk(sky[2] == 0.8 && horizon[1] == 0.4 && ground[0] == 0.2, "environment() reads it back");
  sg.resetEnvironment();
  sg.environment(sky, horizon, ground);
  chk(std::fabs(sky[0] - 0.50) < 1e-12 && std::fabs(horizon[0] - 0.42) < 1e-12 &&
          std::fabs(ground[0] - 0.20) < 1e-12,
      "resetEnvironment: the defaults");

  std::printf("F. another renderer\n");
  vtkRenderer *first = r.renderer();
  {
    SceneRenderer second(sg, 32, 32, /*offscreen=*/true);
    second.render();
    chk(second.renderer()->GetUseImageBasedLighting() &&
            second.renderer()->GetEnvironmentTexture() == sg.environmentTexture(),
        "the environment follows the scene to a new renderer");
    chk(!first->GetUseImageBasedLighting(), "and leaves the old one");
  }

  std::printf("%s (%d failed)\n", fails ? "FAILED" : "OK", fails);
  return fails ? 1 : 0;
}
