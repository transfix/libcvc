// Where does the ~103us per setPosition actually go?
#include <chrono>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <sstream>

template <typename F> double ms(int reps, F f) {
  auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < reps; ++i)
    f(i);
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() /
         reps;
}

int main() {
  cvc::app app;
  SceneGraph sg(app);
  cvc::geometry g;
  cvc::geometry::point_t p;
  p[0] = 0;
  p[1] = 0;
  p[2] = 0;
  g.points().push_back(p);
  auto n = sg.addGraphics("probe", g);

  const int R = 3000;
  printf("  ostringstream only      : %8.1f us\n", 1000.0 * ms(R, [&](int i) {
                                                     std::ostringstream oss;
                                                     oss << 1.0 * i << "," << 2.0 << "," << 3.0;
                                                     volatile auto s = oss.str().size();
                                                     (void)s;
                                                   }));
  printf("  getState(\"position\")     : %8.1f us\n", 1000.0 * ms(R, [&](int i) {
                                                        volatile auto *q = &n->getState("position");
                                                        (void)q;
                                                      }));
  printf("  getState().fullName()   : %8.1f us\n", 1000.0 * ms(R, [&](int i) {
                                                     volatile auto s =
                                                         n->getState("position").fullName().size();
                                                     (void)s;
                                                   }));
  printf("  .value(str) SAME value  : %8.1f us\n",
         1000.0 * ms(R, [&](int i) { n->getState("position").value(std::string("1,2,3")); }));
  printf("  .value(str) CHANGING    : %8.1f us\n",
         1000.0 * ms(R, [&](int i) { n->getState("position").value(std::to_string(i)); }));
  printf("  full setPosition        : %8.1f us\n",
         1000.0 * ms(R, [&](int i) { n->setPosition(0.001 * i, 1.0, 0.0); }));
  return 0;
}
