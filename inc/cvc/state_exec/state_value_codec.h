#ifndef CVC_STATE_EXEC_STATE_VALUE_CODEC_H
#define CVC_STATE_EXEC_STATE_VALUE_CODEC_H

#include <cvc/state_exec/stackless_evaluator.h>
#include <cvc/state_exec/types.h>

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
/// cannot be serialized.
void encode_value(cvc::state& node, const value_t& val);

/// Decode a value_t from a state subtree previously written by encode_value.
/// native_fn markers decode to nil.
value_t decode_value(cvc::state& node);

// ---------------------------------------------------------------------------
// Environment codec
// ---------------------------------------------------------------------------

/// Encode an environment chain into a state subtree.
/// Flattens the scope chain into an indexed list of scope nodes, each
/// containing its bindings.  native_fn bindings are skipped.
void encode_environment(cvc::state& node, const environment_ptr& env);

/// Decode an environment chain from a state subtree.
environment_ptr decode_environment(cvc::state& node);

// ---------------------------------------------------------------------------
// Evaluator-state codec (stackless)
// ---------------------------------------------------------------------------

/// Encode a full evaluator_state snapshot into a state subtree.
void encode_evaluator_state(cvc::state& node, const evaluator_state& es);

/// Decode a full evaluator_state snapshot from a state subtree.
evaluator_state decode_evaluator_state(cvc::state& node);

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_STATE_VALUE_CODEC_H
