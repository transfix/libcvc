// One-shot-per-frame vs a renderer held open, same scene, same frame count.
#include <chrono>
#include <cstdio>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <vtkNew.h>
#include <vtkPNGWriter.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkWindowToImageFilter.h>

static void addQuad(SceneGraph &sg, const char *n, double s, double z) {
  cvc::geometry g;
  const double xs[4] = {-s, s, s, -s}, ys[4] = {-s, -s, s, s};
  for (int i = 0; i < 4; ++i) {
    cvc::geometry::point_t p;
    p[0] = xs[i];
    p[1] = ys[i];
    p[2] = z;
    g.points().push_back(p);
  }
  const int idx[2][3] = {{0, 1, 2}, {0, 2, 3}};
  for (auto &t3 : idx) {
    cvc::geometry::tri_t t;
    t[0] = t3[0];
    t[1] = t3[1];
    t[2] = t3[2];
    g.tris().push_back(t);
  }
  sg.addGraphics(n, g);
}
// the OLD path, verbatim in shape: build context, attach, render, finalize
static void oneShot(SceneGraph &sg, const char *path, int w, int h) {
  vtkNew<vtkRenderer> r;
  vtkNew<vtkRenderWindow> win;
  win->SetOffScreenRendering(1);
  win->AddRenderer(r);
  win->SetSize(w, h);
  sg.setRenderer(r);
  sg.processEvents();
  r->ResetCamera();
  win->Render();
  vtkNew<vtkWindowToImageFilter> w2i;
  w2i->SetInput(win);
  w2i->Update();
  vtkNew<vtkPNGWriter> pw;
  pw->SetFileName(path);
  pw->SetInputConnection(w2i->GetOutputPort());
  pw->Write();
  sg.setRenderer(nullptr);
  win->Finalize();
}
int main(int argc, char **argv) {
  const int N = argc > 1 ? atoi(argv[1]) : 40;
  const int W = 960, H = 540;
  SceneGraph sg;
  for (int i = 0; i < 40; ++i)
    addQuad(sg, (std::string("q") + std::to_string(i)).c_str(), 10 + i, i * 0.5);
  using clk = std::chrono::steady_clock;
  auto t0 = clk::now();
  for (int i = 0; i < N; ++i)
    oneShot(sg, "/tmp/bench_oneshot.png", W, H);
  auto t1 = clk::now();
  {
    SceneRenderer r(sg, W, H, true);
    for (int i = 0; i < N; ++i)
      r.writePNG("/tmp/bench_persist.png");
  }
  auto t2 = clk::now();
  {
    SceneRenderer r(sg, W, H, true);
    for (int i = 0; i < N; ++i)
      (void)r.frameRGB();
  }
  auto t3 = clk::now();
  auto ms = [](auto a, auto b) { return std::chrono::duration<double, std::milli>(b - a).count(); };
  printf("  %d frames @ %dx%d, 40-node scene\n", N, W, H);
  printf("  one-shot per frame (the old path) : %8.1f ms total  %6.1f ms/frame\n", ms(t0, t1),
         ms(t0, t1) / N);
  printf("  SceneRenderer -> PNG              : %8.1f ms total  %6.1f ms/frame   %.1fx faster\n",
         ms(t1, t2), ms(t1, t2) / N, ms(t0, t1) / ms(t1, t2));
  printf("  SceneRenderer -> raw RGB          : %8.1f ms total  %6.1f ms/frame   %.1fx faster\n",
         ms(t2, t3), ms(t2, t3) / N, ms(t0, t1) / ms(t2, t3));
  printf("  implied fps: old %.1f, persistent-PNG %.1f, raw-RGB %.1f\n", 1000.0 * N / ms(t0, t1),
         1000.0 * N / ms(t1, t2), 1000.0 * N / ms(t2, t3));
  return 0;
}
