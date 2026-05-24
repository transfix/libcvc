#ifndef CVC_STATE_EXEC_INTRINSICS_H
#define CVC_STATE_EXEC_INTRINSICS_H

#include <cvc/state_exec/types.h>

#include <functional>
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
struct intrinsics_context {
    scheduler*      sched    = nullptr;   // Owning scheduler
    cvc::state*     root     = nullptr;   // State tree root
    memory_tracker* tracker  = nullptr;   // Memory tracker
    process*        proc     = nullptr;   // Current process
    int             pid      = -1;        // Current process PID
    std::string     uid;                  // Process user identity
    std::string     cluster_id;           // Cluster identity
    std::string     node_id;              // Node identity
};

/// Register all DSL intrinsics into an environment.
///
/// The context pointer must remain valid for the lifetime of any evaluation
/// that uses the returned environment.  Typically, the scheduler creates one
/// context per process and injects intrinsics before execution begins.
void register_intrinsics(environment_ptr env, intrinsics_context* ctx);

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_INTRINSICS_H
