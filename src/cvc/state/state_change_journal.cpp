#include <algorithm>
#include <cvc/state/state_change_journal.h>
#include <sstream>

namespace cvc {

state_payload state_payload::none() { return state_payload(); }

state_payload state_payload::inline_data(const std::vector<unsigned char> &bytes) {
  state_payload payload;
  payload.kind = state_payload_kind::inline_bytes;
  payload.inline_bytes = bytes;
  return payload;
}

state_payload state_payload::blob_ref(const state_blob_ref &ref) {
  state_payload payload;
  payload.kind = state_payload_kind::blob;
  payload.blob = ref;
  return payload;
}

bool state_payload::empty() const {
  return kind == state_payload_kind::none ||
         (kind == state_payload_kind::inline_bytes && inline_bytes.empty()) ||
         (kind == state_payload_kind::blob && blob.digest.empty());
}

std::string to_string(state_mutation_op op) {
  switch (op) {
  case state_mutation_op::set_value:
    return "set_value";
  case state_mutation_op::set_data:
    return "set_data";
  case state_mutation_op::set_comment:
    return "set_comment";
  case state_mutation_op::set_hidden:
    return "set_hidden";
  case state_mutation_op::set_read_only:
    return "set_read_only";
  case state_mutation_op::set_property:
    return "set_property";
  case state_mutation_op::touch:
    return "touch";
  case state_mutation_op::reset_node:
    return "reset_node";
  case state_mutation_op::delete_subtree:
    return "delete_subtree";
  case state_mutation_op::delegate_subtree:
    return "delegate_subtree";
  case state_mutation_op::revoke_delegation:
    return "revoke_delegation";
  }
  return "unknown";
}

state_change_journal::state_change_journal(const std::string &local_node_id)
    : _local_node_id(local_node_id.empty() ? std::string("local") : local_node_id),
      _next_sequence(1) {}

state_mutation state_change_journal::append(const state_mutation &mutation) {
  std::lock_guard<std::mutex> lock(_mutex);

  state_mutation stored = mutation;
  stored.sequence = _next_sequence++;
  if (stored.origin_node_id.empty()) {
    stored.origin_node_id = _local_node_id;
  }
  if (stored.mutation_id.empty()) {
    stored.mutation_id = make_mutation_id(stored);
  }

  _mutations.push_back(stored);
  return stored;
}

std::vector<state_mutation> state_change_journal::snapshot() const {
  std::lock_guard<std::mutex> lock(_mutex);
  return _mutations;
}

std::vector<state_mutation> state_change_journal::replay_after(std::uint64_t sequence) const {
  std::lock_guard<std::mutex> lock(_mutex);

  // append() assigns strictly increasing sequences, so the tail after `sequence` is found by
  // binary search: a drain that runs every frame costs O(new), not O(journal).
  const auto first =
      std::upper_bound(_mutations.begin(), _mutations.end(), sequence,
                       [](std::uint64_t s, const state_mutation &m) { return s < m.sequence; });
  return std::vector<state_mutation>(first, _mutations.end());
}

std::size_t state_change_journal::trim_through(std::uint64_t sequence) {
  std::lock_guard<std::mutex> lock(_mutex);
  const auto last =
      std::upper_bound(_mutations.begin(), _mutations.end(), sequence,
                       [](std::uint64_t s, const state_mutation &m) { return s < m.sequence; });
  const auto n = static_cast<std::size_t>(last - _mutations.begin());
  _mutations.erase(_mutations.begin(), last);
  return n;
}

void state_change_journal::clear() {
  std::lock_guard<std::mutex> lock(_mutex);
  _mutations.clear();
}

std::size_t state_change_journal::size() const {
  std::lock_guard<std::mutex> lock(_mutex);
  return _mutations.size();
}

std::uint64_t state_change_journal::last_sequence() const {
  std::lock_guard<std::mutex> lock(_mutex);
  return _next_sequence == 1 ? 0 : _next_sequence - 1;
}

const std::string &state_change_journal::local_node_id() const { return _local_node_id; }

std::string state_change_journal::make_mutation_id(const state_mutation &mutation) const {
  std::ostringstream output;
  output << mutation.origin_node_id << ':' << mutation.sequence;
  return output.str();
}

} // namespace cvc
