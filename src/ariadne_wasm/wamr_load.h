// wamr_load.h — what WAMR's loader says of a guest's binary, for the WAMR backend
// (wamr_runtime.cpp): whether it loads, with the loader's reason when it doesn't, and whether the
// gas pass and Asyncify have been over it. On WAMR's native API (wasm_export.h, wamr_load.cpp),
// whose declarations can't share a file with the wasm-c-api's (wasm_c_api.h): the two headers
// declare wasm_module_t differently.
#pragma once

#include <string>
#include <string_view>

namespace cvc {
namespace ariadne {
namespace wasm {

struct WamrShape {
  bool metered = false;  // it imports ari.gas and exports the ari_gas global (meter.h)
  bool asyncify = false; // it exports Asyncify's five functions
};

// Load `bytes` with WAMR's loader, which validates as it loads, into `shape`. False, with `error`
// set (the loader's message), for bytes WAMR won't load. The runtime must be running (a wasm-c-api
// engine).
bool wamr_inspect(std::string_view bytes, WamrShape &shape, std::string &error);

// "2.4.4": the WAMR this host links.
std::string wamr_version();

} // namespace wasm
} // namespace ariadne
} // namespace cvc
