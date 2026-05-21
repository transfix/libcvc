#ifndef __CVC_STATE_CHANGE_JOURNAL_H__
#define __CVC_STATE_CHANGE_JOURNAL_H__

#include <cstdint>
#include <cvc/namespace.h>
#include <mutex>
#include <string>
#include <vector>

namespace CVC_NAMESPACE {

enum class state_mutation_op {
  set_value,
  set_data,
  set_comment,
  set_hidden,
  set_read_only,
  touch,
  reset_node,
  delete_subtree,
  delegate_subtree,
  revoke_delegation
};

enum class state_payload_kind { none, inline_bytes, blob };

struct state_blob_ref {
  std::string digest;
  std::uint64_t size_bytes = 0;
  std::string codec;
};

struct state_payload {
  state_payload_kind kind = state_payload_kind::none;
  std::vector<unsigned char> inline_bytes;
  state_blob_ref blob;

  static state_payload none();
  static state_payload inline_data(const std::vector<unsigned char> &bytes);
  static state_payload blob_ref(const state_blob_ref &ref);

  bool empty() const;
};

struct state_mutation {
  std::string cluster_id;
  std::string tree_id;
  std::string origin_node_id;
  std::uint64_t sequence = 0;
  std::string mutation_id;
  std::string path;
  state_mutation_op op = state_mutation_op::set_value;
  std::string type_name;
  std::string string_value;
  state_payload payload;
  bool latest_value_only = false;
};

std::string to_string(state_mutation_op op);

class state_change_journal {
public:
  explicit state_change_journal(const std::string &local_node_id = std::string());

  state_mutation append(const state_mutation &mutation);
  std::vector<state_mutation> snapshot() const;
  std::vector<state_mutation> replay_after(std::uint64_t sequence) const;
  void clear();

  std::size_t size() const;
  std::uint64_t last_sequence() const;
  const std::string &local_node_id() const;

private:
  std::string make_mutation_id(const state_mutation &mutation) const;

  mutable std::mutex _mutex;
  std::string _local_node_id;
  std::uint64_t _next_sequence;
  std::vector<state_mutation> _mutations;
};

} // namespace CVC_NAMESPACE

#endif // __CVC_STATE_CHANGE_JOURNAL_H__
