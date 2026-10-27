// SdlInput::attach on a real VTK window (Windows): SDL wraps the HWND VTK made, a key sent to that
// window reaches SDL -- so a document's on_key sees it -- and still reaches VTK's own interactor,
// so ImGui and the camera keep their keys. Once SDL lets go, VTK alone still gets them. Then the
// path X11, Wayland and macOS take ($CVC_SDLINPUT_VTK_KEYS forces it here): VTK's own key events,
// named as SDL names them, a held key's presses marked as repeats. SKIPs (rc 0) without SDL, off
// Windows, or where no window opens.
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/SdlInput.h>
#include <string>
#include <vector>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#ifdef _WIN32
#include <windows.h>
#endif

namespace {

int failures = 0;

void chk(bool ok, const std::string &what) {
  std::printf("%s %s\n", ok ? "ok  " : "FAIL", what.c_str());
  if (!ok)
    ++failures;
}

#ifdef _WIN32
// The Left arrow as Windows sends it: scan code 0x4B, an extended key.
const LPARAM kLeftDown = 1 | (0x4B << 16) | (1 << 24);
void press_left(HWND hwnd) {
  SendMessageW(hwnd, WM_KEYDOWN, VK_LEFT, kLeftDown);
  SendMessageW(hwnd, WM_KEYUP, VK_LEFT, kLeftDown | (LPARAM(1) << 30) | (LPARAM(1) << 31));
}
#endif

} // namespace

int main() {
#if !defined(CVC_ENABLE_SDL) || !defined(_WIN32)
  std::printf("SKIP: needs a Windows build with SDL\n");
  return 0;
#else
  cvc::app app;
  cvc::gl::SceneGraph sg(app, "sdl_attach");
  cvc::gl::SceneRenderer view(sg, 240, 160, /*offscreen=*/false, "main");
  view.render(); // VTK makes its window at the first render
  vtkRenderWindow *win = view.renderWindow();
  const HWND hwnd = win ? static_cast<HWND>(win->GetGenericWindowId()) : nullptr;
  vtkRenderWindowInteractor *iren = win ? win->GetInteractor() : nullptr;
  if (!hwnd || !iren) {
    std::printf("SKIP: no onscreen VTK window here\n");
    return 0;
  }
  std::vector<std::string> vtk_keys; // what VTK's interactor saw
  auto seen = vtkSmartPointer<vtkCallbackCommand>::New();
  seen->SetClientData(&vtk_keys);
  seen->SetCallback([](vtkObject *caller, unsigned long, void *keys, void *) {
    const char *sym = static_cast<vtkRenderWindowInteractor *>(caller)->GetKeySym();
    static_cast<std::vector<std::string> *>(keys)->push_back(sym ? sym : "");
  });
  iren->AddObserver(vtkCommand::KeyPressEvent, seen);

  cvc::gl::SdlInput in;
  chk(!in.attached(), "not attached before attach()");
  chk(in.attach(win), "attach() wraps VTK's window");
  chk(in.attached(), "and reports it attached");

  press_left(hwnd);
  bool down = false, up = false;
  for (const cvc::gl::InputEvent &e : in.poll()) {
    down |= e.kind == cvc::gl::InputEvent::Kind::KeyDown && e.key == "Left";
    up |= e.kind == cvc::gl::InputEvent::Kind::KeyUp && e.key == "Left";
  }
  chk(down && up, "SDL reports the Left arrow down and up");
  chk(vtk_keys.size() == 1 && vtk_keys[0] == "Left", "VTK's interactor still gets it");

  // A click: SDL sees it, and so does VTK (ImGui's clicks come through VTK's interactor).
  int vtk_presses = 0;
  auto pressed = vtkSmartPointer<vtkCallbackCommand>::New();
  pressed->SetClientData(&vtk_presses);
  pressed->SetCallback(
      [](vtkObject *, unsigned long, void *n, void *) { ++*static_cast<int *>(n); });
  iren->AddObserver(vtkCommand::LeftButtonPressEvent, pressed);
  const LPARAM at = MAKELPARAM(40, 30);
  SendMessageW(hwnd, WM_MOUSEMOVE, 0, at);
  SendMessageW(hwnd, WM_LBUTTONDOWN, MK_LBUTTON, at);
  SendMessageW(hwnd, WM_LBUTTONUP, 0, at);
  bool sdl_click = false;
  for (const cvc::gl::InputEvent &e : in.poll())
    sdl_click |= e.kind == cvc::gl::InputEvent::Kind::MouseButtonDown && e.button == 1;
  chk(sdl_click, "SDL reports the click");
  chk(vtk_presses == 1, "VTK's interactor still gets the click");

  in.shutdown(); // SDL hands the window procedure back
  vtk_keys.clear();
  press_left(hwnd);
  chk(vtk_keys.size() == 1 && vtk_keys[0] == "Left", "after shutdown() VTK alone gets it");
  chk(IsWindow(hwnd) != FALSE, "and VTK's window is still standing");

  // VTK's keys, as on X11: Left held (a second press while down is a repeat), then released;
  // a comma (VTK's "comma") comes out as SDL names it.
  _putenv_s("CVC_SDLINPUT_VTK_KEYS", "1");
  cvc::gl::SdlInput vk;
  chk(vk.attach(win) && vk.attached(), "with $CVC_SDLINPUT_VTK_KEYS, attach() takes VTK's keys");
  SendMessageW(hwnd, WM_KEYDOWN, VK_LEFT, kLeftDown);
  SendMessageW(hwnd, WM_KEYDOWN, VK_LEFT, kLeftDown | (LPARAM(1) << 30)); // the key repeats
  SendMessageW(hwnd, WM_KEYUP, VK_LEFT, kLeftDown | (LPARAM(1) << 30) | (LPARAM(1) << 31));
  std::vector<cvc::gl::InputEvent> got = vk.poll();
  std::string seq;
  for (const cvc::gl::InputEvent &e : got)
    seq += std::string(e.kind == cvc::gl::InputEvent::Kind::KeyDown ? "down" : "up") + ":" + e.key +
           (e.repeat ? "(repeat)" : "") + " ";
  chk(seq == "down:Left down:Left(repeat) up:Left ", "a held key repeats, then goes up: " + seq);
  const LPARAM comma = 1 | (0x33 << 16);
  SendMessageW(hwnd, WM_KEYDOWN, VK_OEM_COMMA, comma);
  SendMessageW(hwnd, WM_KEYUP, VK_OEM_COMMA, comma | (LPARAM(1) << 30) | (LPARAM(1) << 31));
  got = vk.poll();
  chk(!got.empty() && got.front().key == ",",
      "VTK's keysym renamed as SDL's: " + (got.empty() ? std::string("nothing") : got.front().key));
  vk.shutdown();
  vtk_keys.clear();
  press_left(hwnd);
  chk(vk.poll().empty() && vtk_keys.size() == 1, "after shutdown() it observes nothing");
  _putenv_s("CVC_SDLINPUT_VTK_KEYS", "");

  std::printf("cvcgl_sdl_attach: %s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}
