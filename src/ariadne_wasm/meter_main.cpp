// meter_main.cpp — ari_wasm_meter <in.wasm> <out.wasm>: the gas pass (meter.h) as a command, the
// install-time transform's first step (cvc_ariadne_wasm_transform, ariadne_wasm.cmake). A module
// that is metered already is written out as it is. Under Emscripten it runs in node, on the host's
// files (NODERAWFS).

#include "meter.h"

#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>

int main(int argc, char **argv) {
  if (argc != 3) {
    std::fprintf(stderr, "usage: ari_wasm_meter <in.wasm> <out.wasm>\n");
    return 2;
  }
  std::ifstream in(argv[1], std::ios::binary);
  if (!in) {
    std::fprintf(stderr, "ari_wasm_meter: can't read %s\n", argv[1]);
    return 1;
  }
  const std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  cvc::ariadne::wasm::Metered out;
  std::string error;
  if (!cvc::ariadne::wasm::meter(bytes, out, error)) {
    std::fprintf(stderr, "ari_wasm_meter: %s: %s\n", argv[1], error.c_str());
    return 1;
  }
  std::ofstream o(argv[2], std::ios::binary);
  o.write(out.bytes.data(), static_cast<std::streamsize>(out.bytes.size()));
  if (!o) {
    std::fprintf(stderr, "ari_wasm_meter: can't write %s\n", argv[2]);
    return 1;
  }
  return 0;
}
