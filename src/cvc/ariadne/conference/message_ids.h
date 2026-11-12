// message_ids.h — private to the conference sources: one sender's state_message ids.
#pragma once

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <random>
#include <string>

namespace cvc {
namespace ariadne {
namespace conference {

// The message ids of one sender (a relay, a client, a lane session): a random tag of its own, then
// a count. The bus drops an id it has seen from the same origin, so a sender made again under the
// same pid (a host that joined again), or a second one on a shard, must not repeat one. Each
// sender owns one; nothing is shared between senders or apps.
class message_ids {
public:
  message_ids() {
    std::random_device rd;
    const unsigned long long r = (static_cast<unsigned long long>(rd()) << 32) ^ rd();
    char tag[17];
    std::snprintf(tag, sizeof tag, "%016llx", r);
    _tag = tag;
  }
  // "<tag>-<n>", n counting from 1.
  std::string next() { return _tag + "-" + std::to_string(_n.fetch_add(1) + 1); }

private:
  std::string _tag;
  std::atomic<std::uint64_t> _n{0};
};

} // namespace conference
} // namespace ariadne
} // namespace cvc
