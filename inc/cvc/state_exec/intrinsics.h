#ifndef CVC_STATE_EXEC_INTRINSICS_H
#define CVC_STATE_EXEC_INTRINSICS_H

#include <cvc/state_exec/types.h>

#include <functional>
#include <memory>
#include <string>

namespace cvc {
class state;
}

namespace cvc::state_exec {

class scheduler;
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
struct intrinsics_context {
    scheduler*                  sched    = nullptr;   // Non-owning; outlives context
    cvc::state*                 root     = nullptr;   // Non-owning; app-scoped lifetime
    memory_tracker*             tracker  = nullptr;   // Non-owning; scheduler member
    std::shared_ptr<process>    proc;                 // Shared with scheduler
    int                         pid      = -1;        // Current process PID
    std::string                 uid;                  // Process user identity
    std::string                 cluster_id;           // Cluster identity
    std::string                 node_id;              // Node identity
};

/// Register all DSL intrinsics into an environment.
///
/// The context pointer must remain valid for the lifetime of any evaluation
/// that uses the returned environment.  Typically, the scheduler creates one
/// context per process and injects intrinsics before execution begins.
void register_intrinsics(environment_ptr env, intrinsics_context* ctx);

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_INTRINSICS_H
