// pycvc_exec.cpp — Exec facade implementation. The whole libcvc-touching body
// is guarded by CVC_STATE_EXEC (PUBLIC compile def on the `cvc` target); when
// off, ExecImpl is an empty shell and the methods throw.
#include "pycvc_exec.h"

#include "pycvc_context.h"

#include <stdexcept>

#ifdef CVC_STATE_EXEC

#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_exec/builtins.h>
#include <cvc/core/state_exec/intrinsics.h>
#include <cvc/core/state_exec/process.h>
#include <cvc/core/state_exec/scheduler.h>
#include <cvc/core/state_exec/types.h>

#include <string>
#include <variant>

namespace pycvc {

namespace {
// Render a DSL result variant for Python. Strings come back raw (unquoted) so
// values round-trip through the state tree; nil becomes ""; everything else
// uses the DSL's own printed form via state_exec::to_string.
std::string render_result(const cvc::state_exec::value_t &v) {
  if (const auto *s = std::get_if<std::string>(&v.v))
    return *s;
  if (v.is_nil())
    return "";
  return cvc::state_exec::to_string(v);
}
} // namespace

// Owns everything an evaluation needs. The intrinsics capture &ictx (a raw
// pointer), so ictx must outlive any run — holding it here (facade-lifetime)
// satisfies that. Mirrors the C++ tests' make_exec_env().
struct ExecImpl {
  cvc::state *root = nullptr;
  cvc::state_exec::scheduler sched;
  cvc::state_exec::intrinsics_context ictx;
  cvc::state_exec::process_ptr host_proc;
  cvc::state_exec::environment_ptr env;
};

Exec::Exec() : impl_(std::make_shared<ExecImpl>()) {
  using namespace cvc::state_exec;
  impl_->root = &cvc::state::instance(process_ctx());
  impl_->sched.set_watch_root(impl_->root);

  impl_->host_proc = make_process();
  impl_->host_proc->pid = 0;
  impl_->host_proc->status = process_status::ready;

  impl_->ictx.sched = &impl_->sched;
  impl_->ictx.root = impl_->root;
  impl_->ictx.proc = impl_->host_proc;
  impl_->ictx.pid = 0;
  impl_->ictx.uid = "pycvc";
  impl_->ictx.cluster_id = "local";
  impl_->ictx.node_id = "local";

  impl_->env = builtins::make_default_environment();
  register_intrinsics(impl_->env, &impl_->ictx);
}

Exec::~Exec() = default;

std::string Exec::run_program(const std::string &src) {
  using namespace cvc::state_exec;
  execute_options opts;
  opts.name = "pycvc";
  opts.env = impl_->env;
  int pid = impl_->sched.execute(src, opts); // may throw on parse error
  impl_->sched.run();
  std::optional<value_t> result = impl_->sched.get_result(pid);
  if (!result.has_value())
    throw std::runtime_error("Exec.run_program: program produced no result (pid " +
                             std::to_string(pid) + ")");
  return render_result(*result);
}

} // namespace pycvc

#else // !CVC_STATE_EXEC

namespace pycvc {

struct ExecImpl {};

Exec::Exec() {
  throw std::runtime_error("pycvc.Exec: this libcvc build was compiled without state_exec");
}
Exec::~Exec() = default;

std::string Exec::run_program(const std::string &) {
  throw std::runtime_error("pycvc.Exec: this libcvc build was compiled without state_exec");
}

} // namespace pycvc

#endif // CVC_STATE_EXEC
