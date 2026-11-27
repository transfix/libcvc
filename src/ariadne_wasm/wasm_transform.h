// wasm_transform.h — the install-time transform's binaryen steps in process (G3,
// docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §8.5), for the WAMR backend (wamr_runtime.cpp), which
// runs only transformed guests: a guest's WebAssembly text as a binary, and a metered guest with
// Asyncify around its `ari` imports. They are the steps cvc_ariadne_wasm_transform runs through
// wasm-opt (ariadne_wasm.cmake), on binaryen's C++ API and with the same features and passes, so a
// guest transformed here is the one the install makes:
//
//   wasm-opt <in.wat> <features> -o <binary>                                    (wat_to_wasm)
//   ari_wasm_meter <binary> <metered>                                           (meter.h)
//   wasm-opt <metered> <features> --asyncify --pass-arg=asyncify-imports@ari.* -O1 -o <out>
//                                                                                (asyncify)
//
// Built where binaryen is found (cvc_ariadne_wasm_host, CVC_ARIADNE_WASM_BINARYEN). Each call
// works on a module of its own, with its own pass options, so any thread may call them at once.
#pragma once

#include <string>
#include <string_view>

namespace cvc {
namespace ariadne {
namespace wasm {

// `text`, a module in the WebAssembly text format, as a binary in `out`, with every feature the
// parser knows. False, with `error` set (the parser's message and where), for text that doesn't
// parse. The module isn't validated: the runtime's loader validates what it loads.
bool wat_to_wasm(std::string_view text, std::string &out, std::string &error);

// `metered`, a binary module the gas pass has metered (meter.h), with binaryen's Asyncify around
// every `ari` import, ari.gas among them, then optimized at -O1, in `out`. False, with `error` set,
// for a module binaryen can't read or that isn't valid.
bool asyncify(std::string_view metered, std::string &out, std::string &error);

} // namespace wasm
} // namespace ariadne
} // namespace cvc
