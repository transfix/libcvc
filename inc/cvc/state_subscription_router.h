#ifndef __CVC_STATE_SUBSCRIPTION_ROUTER_H__
#define __CVC_STATE_SUBSCRIPTION_ROUTER_H__

#include <cvc/namespace.h>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace CVC_NAMESPACE {

using state_subscription_id = std::uint64_t;

struct state_subscription {
  state_subscription_id id = 0;
  std::string path_prefix;
  bool include_descendants = true;
};

class state_subscription_router {
public:
  state_subscription_router();

  state_subscription_id subscribe(const std::string &path_prefix,
                                  bool include_descendants = true);
  bool unsubscribe(state_subscription_id id);
  std::vector<state_subscription> subscriptions_for(const std::string &path) const;
  void clear();

  std::size_t size() const;

private:
  static std::string normalize_path(const std::string &path);
  static std::vector<std::string> split_path(const std::string &path);
  struct trie_node {
    std::vector<state_subscription> subscriptions;
    std::map<std::string, std::unique_ptr<trie_node>> children;
  };
  trie_node &find_or_create_node(const std::string &path_prefix);
  trie_node *find_node(const std::string &path_prefix);
  const trie_node *find_node(const std::string &path_prefix) const;

  mutable std::mutex _mutex;
  state_subscription_id _next_id;
  trie_node _root;
  std::unordered_map<state_subscription_id, state_subscription> _subscriptions;
};

} // namespace CVC_NAMESPACE

#endif // __CVC_STATE_SUBSCRIPTION_ROUTER_H__
