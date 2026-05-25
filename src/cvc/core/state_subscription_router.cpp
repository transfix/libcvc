#include <algorithm>
#include <cvc/core/state_subscription_router.h>

namespace cvc {

state_subscription_router::state_subscription_router() : _next_id(1) {}

state_subscription_id state_subscription_router::subscribe(const std::string &path_prefix,
                                                           bool include_descendants) {
  std::lock_guard<std::mutex> lock(_mutex);

  state_subscription subscription;
  subscription.id = _next_id++;
  subscription.path_prefix = normalize_path(path_prefix);
  subscription.include_descendants = include_descendants;
  _subscriptions[subscription.id] = subscription;
  find_or_create_node(subscription.path_prefix).subscriptions.push_back(subscription);
  return subscription.id;
}

bool state_subscription_router::unsubscribe(state_subscription_id id) {
  std::lock_guard<std::mutex> lock(_mutex);

  auto subscription_iter = _subscriptions.find(id);
  if (subscription_iter == _subscriptions.end()) {
    return false;
  }

  trie_node *node = find_node(subscription_iter->second.path_prefix);
  if (node != nullptr) {
    auto new_end = std::remove_if(
        node->subscriptions.begin(), node->subscriptions.end(),
        [id](const state_subscription &subscription) { return subscription.id == id; });
    node->subscriptions.erase(new_end, node->subscriptions.end());
  }
  _subscriptions.erase(subscription_iter);
  return true;
}

std::vector<state_subscription>
state_subscription_router::subscriptions_for(const std::string &path) const {
  std::lock_guard<std::mutex> lock(_mutex);

  std::string normalized_path = normalize_path(path);
  std::vector<std::string> components = split_path(normalized_path);
  std::vector<state_subscription> matches;

  for (const state_subscription &subscription : _root.subscriptions) {
    matches.push_back(subscription);
  }

  const trie_node *node = &_root;
  for (std::size_t component_index = 0; component_index < components.size(); ++component_index) {
    auto child_iter = node->children.find(components[component_index]);
    if (child_iter == node->children.end()) {
      break;
    }
    node = child_iter->second.get();

    bool exact_path = component_index == components.size() - 1;
    for (const state_subscription &subscription : node->subscriptions) {
      if (exact_path || subscription.include_descendants) {
        matches.push_back(subscription);
      }
    }
  }

  return matches;
}

void state_subscription_router::clear() {
  std::lock_guard<std::mutex> lock(_mutex);
  _root = trie_node();
  _subscriptions.clear();
}

std::size_t state_subscription_router::size() const {
  std::lock_guard<std::mutex> lock(_mutex);
  return _subscriptions.size();
}

std::string state_subscription_router::normalize_path(const std::string &path) {
  std::size_t begin = 0;
  while (begin < path.size() && path[begin] == '.') {
    ++begin;
  }

  std::size_t end = path.size();
  while (end > begin && path[end - 1] == '.') {
    --end;
  }

  return path.substr(begin, end - begin);
}

std::vector<std::string> state_subscription_router::split_path(const std::string &path) {
  std::vector<std::string> components;
  std::size_t begin = 0;
  while (begin < path.size()) {
    std::size_t separator = path.find('.', begin);
    std::size_t end = separator == std::string::npos ? path.size() : separator;
    if (end > begin) {
      components.push_back(path.substr(begin, end - begin));
    }
    if (separator == std::string::npos) {
      break;
    }
    begin = separator + 1;
  }
  return components;
}

state_subscription_router::trie_node &
state_subscription_router::find_or_create_node(const std::string &path_prefix) {
  trie_node *node = &_root;
  std::vector<std::string> components = split_path(path_prefix);
  for (const std::string &component : components) {
    std::unique_ptr<trie_node> &child = node->children[component];
    if (!child) {
      child.reset(new trie_node());
    }
    node = child.get();
  }
  return *node;
}

state_subscription_router::trie_node *
state_subscription_router::find_node(const std::string &path_prefix) {
  return const_cast<trie_node *>(
      static_cast<const state_subscription_router *>(this)->find_node(path_prefix));
}

const state_subscription_router::trie_node *
state_subscription_router::find_node(const std::string &path_prefix) const {
  const trie_node *node = &_root;
  std::vector<std::string> components = split_path(path_prefix);
  for (const std::string &component : components) {
    auto child_iter = node->children.find(component);
    if (child_iter == node->children.end()) {
      return nullptr;
    }
    node = child_iter->second.get();
  }
  return node;
}

} // namespace cvc
