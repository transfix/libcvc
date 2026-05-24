#include <algorithm>
#include <cvc/state_exec/memory_tracker.h>

namespace cvc::state_exec {

void memory_tracker::add_bytes(int pid, std::size_t bytes) {
  current_[pid] += bytes;
  if (current_[pid] > peak_[pid])
    peak_[pid] = current_[pid];
}

void memory_tracker::sub_bytes(int pid, std::size_t bytes) {
  auto &cur = current_[pid];
  cur = (bytes > cur) ? 0 : cur - bytes;
}

void memory_tracker::record_write(int pid, uintptr_t id, std::size_t bytes) {
  auto it = ownership_.find(id);
  if (it != ownership_.end()) {
    // Node already owned — credit old owner, charge new owner
    auto &entry = it->second;
    sub_bytes(entry.pid, entry.bytes);
    entry.pid = pid;
    entry.bytes = bytes;
  } else {
    ownership_[id] = {pid, bytes};
  }
  add_bytes(pid, bytes);
}

void memory_tracker::record_delete(uintptr_t id) {
  auto it = ownership_.find(id);
  if (it == ownership_.end())
    return;
  sub_bytes(it->second.pid, it->second.bytes);
  ownership_.erase(it);
}

std::size_t memory_tracker::current_bytes(int pid) const {
  auto it = current_.find(pid);
  return it != current_.end() ? it->second : 0;
}

std::size_t memory_tracker::peak_bytes(int pid) const {
  auto it = peak_.find(pid);
  return it != peak_.end() ? it->second : 0;
}

void memory_tracker::release_ownership(int pid) {
  for (auto &[id, entry] : ownership_) {
    if (entry.pid == pid) {
      entry.pid = 0; // Reassign to unowned
    }
  }
  current_[pid] = 0;
}

void memory_tracker::fork_ownership(int parent_pid, int child_pid,
                                    const std::vector<uintptr_t> &child_node_ids) {
  for (auto id : child_node_ids) {
    auto it = ownership_.find(id);
    if (it != ownership_.end() && it->second.pid == parent_pid) {
      // Create new entry for child with same byte count
      add_bytes(child_pid, it->second.bytes);
    }
  }
}

void memory_tracker::clear() {
  ownership_.clear();
  current_.clear();
  peak_.clear();
}

} // namespace cvc::state_exec
