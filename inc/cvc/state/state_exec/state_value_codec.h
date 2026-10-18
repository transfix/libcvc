/**
 * @file state_value_codec.h
 * @brief Bidirectional codec between state_exec value_t and cvc::state.
 *
 * Serialises DSL values (including lambdas and evaluator state) into a
 * cvc::state subtree and deserialises them back.  Used for process
 * checkpointing, migration, and persistent storage of DSL programs.
 */
#ifndef CVC_STATE_EXEC_STATE_VALUE_CODEC_H
#define CVC_STATE_EXEC_STATE_VALUE_CODEC_H

#include <cvc/state/state_exec/stackless_evaluator.h>
#include <cvc/state/state_exec/types.h>
#include <string>

namespace cvc {
class state;
} // namespace cvc

namespace cvc::state_exec {

// ---------------------------------------------------------------------------
// Value codec — round-trip value_t through cvc::state subtrees
// ---------------------------------------------------------------------------

/// Encode a value_t into a state subtree.
/// native_fn values are stored as a marker ("__native__") since functions
/// cannot be serialized. The scopes the value's closures capture are written once each,
/// in an `__envs__` table under `node` that the closures refer to, so closures that share a
/// scope share it again when decoded, and a closure its own scope binds is no cycle.
void encode_value(cvc::state &node, const value_t &val);

/// Decode a value_t from a state subtree previously written by encode_value.
/// native_fn markers decode to nil.
value_t decode_value(cvc::state &node);

// ---------------------------------------------------------------------------
// Environment codec
// ---------------------------------------------------------------------------

/// Encode an environment chain into a state subtree: `node` refers to an entry of an
/// `__envs__` table under it, and each scope in the chain (and each scope a closure bound
/// in one captures) is an entry with its bindings and a reference to its enclosing scope.
/// native_fn bindings are skipped.
void encode_environment(cvc::state &node, const environment_ptr &env);

/// Decode an environment chain from a state subtree, written by encode_environment (or, in
/// an older snapshot, as an inline list of scopes).
environment_ptr decode_environment(cvc::state &node);

// ---------------------------------------------------------------------------
// Evaluator-state codec (stackless)
// ---------------------------------------------------------------------------

/// Encode a full evaluator_state snapshot into a state subtree.
void encode_evaluator_state(cvc::state &node, const evaluator_state &es);

/// Decode a full evaluator_state snapshot from a state subtree.
evaluator_state decode_evaluator_state(cvc::state &node);

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_STATE_VALUE_CODEC_H
