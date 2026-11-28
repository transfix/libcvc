// wamr_load.cpp — see the header.

#include "wamr_load.h"

#include "meter.h"

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <vector>
#include <wasm_export.h>

namespace cvc {
namespace ariadne {
namespace wasm {

bool wamr_inspect(std::string_view bytes, WamrShape &shape, std::string &error) {
  if (bytes.size() > UINT32_MAX) {
    error = "the module is over 4 GiB";
    return false;
  }
  // The loader may write into the bytes it loads, and keeps them while the module lives.
  std::vector<std::uint8_t> copy(bytes.begin(), bytes.end());
  char why[256] = {0};
  wasm_module_t m =
      wasm_runtime_load(copy.data(), static_cast<std::uint32_t>(copy.size()), why, sizeof why);
  if (!m) {
    error = why[0] ? why : "WAMR can't load the module";
    return false;
  }
  bool gas_import = false, gas_global = false;
  const std::int32_t imports = wasm_runtime_get_import_count(m);
  for (std::int32_t i = 0; i < imports; ++i) {
    wasm_import_t im{};
    wasm_runtime_get_import_type(m, i, &im);
    if (im.kind == WASM_IMPORT_EXPORT_KIND_FUNC && im.module_name && im.name &&
        kGasModule == im.module_name && kGasImport == im.name)
      gas_import = true;
  }
  static constexpr std::string_view kAsyncify[] = {"asyncify_start_unwind", "asyncify_stop_unwind",
                                                   "asyncify_start_rewind", "asyncify_stop_rewind",
                                                   "asyncify_get_state"};
  std::size_t asyncify = 0;
  const std::int32_t exports = wasm_runtime_get_export_count(m);
  for (std::int32_t i = 0; i < exports; ++i) {
    wasm_export_t ex{};
    wasm_runtime_get_export_type(m, i, &ex);
    if (!ex.name)
      continue;
    const std::string_view name = ex.name;
    if (ex.kind == WASM_IMPORT_EXPORT_KIND_GLOBAL && name == kGasGlobal)
      gas_global = true;
    else if (ex.kind == WASM_IMPORT_EXPORT_KIND_FUNC &&
             std::find(std::begin(kAsyncify), std::end(kAsyncify), name) != std::end(kAsyncify))
      ++asyncify;
  }
  wasm_runtime_unload(m);
  shape.metered = gas_import && gas_global;
  shape.asyncify = asyncify == std::size(kAsyncify);
  return true;
}

std::string wamr_version() {
  std::uint32_t major = 0, minor = 0, patch = 0;
  wasm_runtime_get_version(&major, &minor, &patch);
  return std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(patch);
}

} // namespace wasm
} // namespace ariadne
} // namespace cvc
