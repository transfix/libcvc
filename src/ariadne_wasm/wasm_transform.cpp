// wasm_transform.cpp — see the header. On binaryen's C++ API (the cvcpkg `binaryen` package's
// include/binaryen), whose failures are exceptions and Err results; its C API exits the process
// on a parse error (Fatal()). Built with -fno-rtti where binaryen is (its CMakeLists.txt builds it
// so on every compiler but MSVC), so the classes this file instantiates match the library's.

#include "wasm_transform.h"

#include "parser/wat-parser.h"
#include "pass.h"
#include "support/result.h"
#include "wasm-binary.h"
#include "wasm-validator.h"
#include "wasm.h"

#include <exception>
#include <vector>

namespace cvc {
namespace ariadne {
namespace wasm {

namespace {

// The features cvc_ariadne_wasm_transform turns on (CVC_WASM_GUEST_FEATURES): the ones wasmtime
// and the browsers run, and none only some do (threads, GC, exception handling, memory64).
::wasm::FeatureSet guest_features() {
  using F = ::wasm::FeatureSet;
  return F(F::SignExt | F::MutableGlobals | F::TruncSat | F::BulkMemory | F::BulkMemoryOpt |
           F::Multivalue | F::ReferenceTypes | F::SIMD | F::ExtendedConst | F::TailCall);
}

std::string write(::wasm::Module &m, const ::wasm::PassOptions &options) {
  ::wasm::BufferWithRandomAccess buffer;
  ::wasm::WasmBinaryWriter writer(&m, buffer, options);
  writer.write();
  return std::string(buffer.begin(), buffer.end());
}

} // namespace

bool wat_to_wasm(std::string_view text, std::string &out, std::string &error) {
  try {
    ::wasm::Module m;
    // Every feature the parser knows, as wasmtime's reads the text: what the module may use is the
    // runtime's to say, and its loader validates the binary (and the gas pass refuses what it
    // can't meter, GC among it, by name).
    m.features = ::wasm::FeatureSet::All;
    if (auto *err = ::wasm::WATParser::parseModule(m, text).getErr()) {
      error = err->msg;
      return false;
    }
    // Written with the guests' features where the module keeps to them, though: binaryen writes a
    // type as those features have it (Type::asWrittenGivenFeatures), so a ref.func's (ref $t) is
    // written as funcref, not as the typed reference that WAMR, which has no GC, refuses. A module
    // that uses more (a GC type) is written as it is, for the loader or the gas pass to refuse by
    // name.
    m.features = guest_features();
    if (!::wasm::WasmValidator().validate(m, ::wasm::WasmValidator::Globally |
                                                 ::wasm::WasmValidator::Quiet))
      m.features = ::wasm::FeatureSet::All;
    out = write(m, ::wasm::PassOptions());
    return true;
  } catch (const ::wasm::ParseException &e) {
    error = e.text;
  } catch (const std::exception &e) {
    error = e.what();
  }
  if (error.empty())
    error = "the text doesn't parse";
  return false;
}

bool asyncify(std::string_view metered, std::string &out, std::string &error) {
  try {
    ::wasm::Module m;
    m.features = guest_features();
    const std::vector<char> input(metered.begin(), metered.end());
    ::wasm::WasmBinaryReader reader(m, m.features, input);
    reader.read();
    if (!::wasm::WasmValidator().validate(m, ::wasm::WasmValidator::Globally |
                                                 ::wasm::WasmValidator::Quiet)) {
      error = "binaryen finds the metered module invalid";
      return false;
    }
    // wasm-opt's -O1 sets the optimize level for every pass on its command line, Asyncify's own
    // optimization of the code it instruments among them, then runs the default pipeline after.
    ::wasm::PassOptions options;
    options.optimizeLevel = 1;
    options.shrinkLevel = 0;
    options.arguments["asyncify-imports"] = "ari.*";
    {
      ::wasm::PassRunner runner(&m, options);
      runner.add("asyncify");
      runner.run();
    }
    {
      ::wasm::PassRunner runner(&m, options);
      runner.addDefaultOptimizationPasses();
      runner.run();
    }
    out = write(m, options);
    return true;
  } catch (const ::wasm::ParseException &e) {
    error = e.text;
  } catch (const std::exception &e) {
    error = e.what();
  }
  if (error.empty())
    error = "binaryen can't read the module";
  return false;
}

} // namespace wasm
} // namespace ariadne
} // namespace cvc
