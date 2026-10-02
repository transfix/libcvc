/**
 * @file intrinsics.h
 * @brief State-tree and system intrinsic functions for the DSL.
 *
 * Provides 35 intrinsic functions that bridge DSL programs to the
 * cvc::state tree (get/set/delete/exists/keys/subscribe/wait-for),
 * scheduler operations (spawn/send/recv/self/sleep/yield/exit/ps/kill),
 * and I/O (print/println/format/error/type-of/to-string/gensym).
 */
#ifndef CVC_STATE_EXEC_INTRINSICS_H
#define CVC_STATE_EXEC_INTRINSICS_H

#include <cvc/core/state_exec/types.h>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cvc {
class state;
}

namespace cvc::state_exec {

class scheduler_base;
class memory_tracker;
struct process;

/// Runtime context available to DSL intrinsics.
///
/// Each process receives an intrinsics_context binding it to the scheduler,
/// state tree root, and its own process record.  Intrinsics capture a
/// pointer to this context via closure.
///
/// Ownership model:
///   - sched, root, tracker: non-owning raw pointers to objects whose
///     lifetime is guaranteed to exceed the context (the scheduler owns
///     the tracker, and the cvc::app owns the state root).
///   - proc: shared_ptr so the process survives even if the scheduler
///     removes it from its map (e.g. after a kill); this prevents
///     dangling-pointer bugs on map rehash or process removal.
// §12 channel enforcement (the runtime backstop for DYNAMIC channel names the load-time lint can't
// see). A host (the Ariadne Runtime) installs this on each action/resident context from a
// document's channels:/lint: declaration. When `enforce` (strict mode), a msg-* is refused unless
// its channel is a '#'-runtime channel, a declared channel, a declared app-root-global (for a
// '/'-ref), or granted via a planted transparent-link (throw, caught fail-safe). In warn/off mode
// the host leaves `enforce` false — the load lint already surfaced those. Empty/absent = permissive
// (non-Ariadne state_exec, or a document with no channels: block).
struct channel_policy {
  bool enforce = false;              // strict: throw on an undeclared channel
  bool quiet = false;                // advisory; the load-time lint owns the "relaxed" notices
  std::vector<std::string> declared; // channel names the document declared
  std::vector<std::string> global;   // declared app-root-globals (the '/'-escape allowlist)
};

// A per-DOCUMENT resource scope, threaded through intrinsics_context by a non-owning pointer. The
// HOST (e.g. the Ariadne Runtime) owns exactly ONE per live document and tears it down during
// document teardown — while the app and its state tree are still alive — so resources an intrinsic
// opens during the document's life (streams, handles, …) are destroyed deterministically WITH the
// document, not at process exit. This is the non-singleton alternative to a process-static owning
// table: ownership follows the owning object's lifetime, never a global. A context whose host
// installs no scope (e.g. the load-time init: lane) leaves `document` null; such an intrinsic must
// fail cleanly rather than leak a resource with no owner.
//
// Resources live in typed, named slots. slot<T>(key, args…) get-or-creates a T owned by the scope
// and returns a stable pointer to it; every call with the SAME key returns the SAME object (a key
// reused with a different T returns nullptr — a programming error, caught without RTTI via a
// per-type tag). On clear()/destruction the slots are destroyed in REVERSE creation order, each
// resource's destructor running while the owning document/app is still alive.
class document_scope {
public:
  document_scope() = default;
  ~document_scope() { clear(); }
  document_scope(const document_scope &) = delete;
  document_scope &operator=(const document_scope &) = delete;

  template <class T, class... Args> T *slot(const std::string &key, Args &&...args) {
    std::lock_guard<std::mutex> lk(mu_);
    auto it = slots_.find(key);
    if (it != slots_.end())
      return it->second.tag == type_tag<T>() ? static_cast<T *>(it->second.obj.get()) : nullptr;
    auto obj = std::make_shared<T>(std::forward<Args>(args)...);
    T *raw = obj.get();
    slots_.emplace(key, entry{std::static_pointer_cast<void>(std::move(obj)), type_tag<T>()});
    order_.push_back(key);
    return raw;
  }

  // Destroy every slot now (reverse creation order), running each resource's destructor while the
  // owner is still alive. Idempotent; the destructor calls it. Resources are released OUTSIDE the
  // lock — a resource dtor may re-enter state/scheduler.
  void clear() {
    std::vector<std::shared_ptr<void>> victims;
    {
      std::lock_guard<std::mutex> lk(mu_);
      victims.reserve(order_.size());
      for (auto it = order_.rbegin(); it != order_.rend(); ++it) {
        auto s = slots_.find(*it);
        if (s != slots_.end())
          victims.push_back(std::move(s->second.obj));
      }
      slots_.clear();
      order_.clear();
    }
  } // victims' destructors run here, after the lock is released

private:
  template <class T> static const void *type_tag() {
    static const char t = 0; // unique address per instantiated T — an RTTI-free type identity
    return &t;
  }
  struct entry {
    std::shared_ptr<void> obj;
    const void *tag;
  };
  std::mutex mu_;
  std::vector<std::string> order_; // creation order, for LIFO teardown
  std::unordered_map<std::string, entry> slots_;
};

struct intrinsics_context {
  scheduler_base *sched = nullptr;   // Non-owning; outlives context (sync or async scheduler)
  cvc::state *root = nullptr;        // Non-owning; app-scoped lifetime
  memory_tracker *tracker = nullptr; // Non-owning; scheduler member
  std::shared_ptr<process> proc;     // Shared with scheduler
  int pid = -1;                      // Current process PID
  std::string uid;                   // Process user identity
  std::string cluster_id;            // Cluster identity
  std::string node_id;               // Node identity
  std::string root_path;             // Chroot path (empty = full tree)
  const channel_policy *channels = nullptr; // §12 channel enforcement (non-owning; host-installed)
  document_scope *document = nullptr; // Per-document resource owner (non-owning; host-installed)

  // State-watch connection registry (opaque — managed by intrinsics impl)
  struct watch_entry {
    std::string path;
    value_t handler;
    std::function<void()> disconnect; // Disconnects from boost signal
  };
  std::unordered_map<int, watch_entry> watches;
};

/// Register all DSL intrinsics into an environment.
///
/// The context pointer must remain valid for the lifetime of any evaluation
/// that uses the returned environment.  Typically, the scheduler creates one
/// context per process and injects intrinsics before execution begins.
void register_intrinsics(environment_ptr env, intrinsics_context *ctx);

/// Resolve `root_path` against `tree_root` and apply chroot to `ctx`.
///
/// If `root_path` is non-empty, sets ctx->root to the subtree node at
/// that path (creating it if needed) and records ctx->root_path.
/// If `root_path` is empty, ctx->root stays at `tree_root`.
///
/// Call this after assigning ctx->root = &tree_root and before
/// register_intrinsics().
void apply_chroot(intrinsics_context &ctx, cvc::state &tree_root, const std::string &root_path);

/// §12 channel scoping: the message-channel key for `channel` under chroot `root_path`, mirroring
/// how state paths are chrooted. This is the PURE-STRING form (no state-tree walk, so it is
/// thread-safe) for HOST posters that must post to the same key a scoped `(msg-recv …)` resolves —
/// e.g. a compute-pool worker calling exec_scheduler().post_message(resolve_channel_key(root_path,
/// "nav.done"), …), where root_path comes from the action's intrinsics_context.root_path.
/// Rules (identical to the intrinsic-side resolver, minus grant-link following which needs the
/// tree): a '#'-bearing channel (the runtime's own tick/key/pointer channels) and an empty
/// root_path return `channel` verbatim (backward-compatible); a leading '/' is an app-root-global
/// escape (returned with the '/' stripped); otherwise the channel is private to the document at
/// "<root_path>.channels.<name>". A host that needs to post to a link-GRANTED channel must resolve
/// on the scheduler thread (the intrinsic path does that) and capture the string — never walk the
/// tree off-thread.
std::string resolve_channel_key(const std::string &root_path, const std::string &channel);

// §13.8 async park primitive — suspend the CURRENTLY-RUNNING process until a value is delivered on
// scheduler channel `ch`, returning a nil placeholder that the scheduler's delivery
// (deliver_to_receivers) patches with the real value in the ENCLOSING expression. This is the
// msg-recv protocol, factored so `await` and host async verbs (e.g. an (http-get) that self-parks)
// share ONE audited suspend path — no evaluator change needed. `ch` must be an ALREADY-RESOLVED
// scheduler key (the caller applies channel policy/scoping; a '#'-runtime channel is exempt). The
// running process is named by `proc` + `pid` (from sched->current_process()/current_pid(), with the
// ctx->proc / ctx->pid fallback for unit-test contexts). Fast-paths a buffered inbox / pending so a
// value that beat the park is returned without suspending. THROWS if the process cannot be
// suspended, or if this is a top-level call with no enclosing frame to receive the delivered value
// (which would otherwise leave a done-and-waiting zombie).
value_t park_on_channel(scheduler_base *sched, process *proc, int pid, const std::string &ch);

// §13.8 futures — a future handle is a tagged one-key dict {"__future__": "<reply-channel>"} (a
// dict_ptr; NO new value_t alternative, so no codec/serialization ripple). An async producer such
// as (http-get-async) returns make_future(chan); (await <future>) and (msg-recv <future>) unwrap it
// with future_channel_of and park on the channel, while a plain value / non-future dict yields
// nullopt (treated as an already-settled value). make_future + future_channel_of are the single
// shared convention so producers and consumers never disagree on the tag.
value_t make_future(const std::string &channel);
std::optional<std::string> future_channel_of(const value_t &v);

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_INTRINSICS_H
