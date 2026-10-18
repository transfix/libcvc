#include <cvc/state/state_list.h>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace cvc {

namespace {

// One node's value and data(). data() is copied (and so signaled) only when either side has some.
void copy_node(state &dst, state &src) {
  dst.value(src.value());
  boost::any d = src.data();
  if (!d.empty() || !dst.data().empty())
    dst.data(d);
}

// Make `dst` hold what `src` holds, subtree and all, when an element moves to the next or the
// previous index: src's value and data, and those of each node under it, are written at the same
// path under dst, and the nodes under dst that src lacks are reset (with signals). Nodes are
// kept, never dropped, as pop_back() keeps the element it clears: callers may hold references
// into an element and observers may be connected to its nodes.
//
// This used to be dst.ptree(src.ptree()). ptree() keys each node by its full path from the root
// and ptree(pt) applies those keys relative to dst, so that wrote src's subtree under
// dst.<src's full path> and left dst's own nodes as they were. data() was not copied either:
// the isData<boost::any>() check before it is false for any data that is not itself a boost::any.
void copy_element(state &dst, state &src) {
  copy_node(dst, src);
  const std::size_t src_skip = src.fullName().size() + 1; // children() names are full names
  for (const std::string &full : src.children()) {
    const std::string rel = full.substr(src_skip);
    if (state *s = src.findDescendant(rel))
      copy_node(dst(rel), *s);
  }
  // children() lists in pre-order, so the descendants of a node src lacks follow it; src lacks
  // them too, and resetting the node reset them.
  const std::size_t dst_skip = dst.fullName().size() + 1;
  std::string cleared; // the last reset node's path and a separator
  for (const std::string &full : dst.children()) {
    const std::string rel = full.substr(dst_skip);
    if (!cleared.empty() && rel.compare(0, cleared.size(), cleared) == 0)
      continue;
    if (src.findDescendant(rel))
      continue;
    if (state *d = dst.findDescendant(rel))
      d->reset(true, true);
    cleared = rel + state::SEPARATOR;
  }
}

} // namespace

state_list::state_list(state &node, std::size_t pad_width) : _node(node), _pad_width(pad_width) {
  // Ensure the __len__ node exists with a default of 0 if absent.
  auto &len_node = _node(len_key);
  if (!len_node.initialized())
    len_node.value("0");
}

std::string state_list::index_key(std::size_t i) const {
  std::ostringstream oss;
  oss << std::setw(static_cast<int>(_pad_width)) << std::setfill('0') << i;
  return oss.str();
}

std::size_t state_list::size() const { return _node(len_key).value<std::size_t>(); }

bool state_list::empty() const { return size() == 0; }

void state_list::set_size(std::size_t n) { _node(len_key).value(n); }

state &state_list::push_back() {
  auto n = size();
  auto &child = _node(index_key(n));
  set_size(n + 1);
  return child;
}

state &state_list::at(std::size_t index) {
  if (index >= size())
    throw std::out_of_range("state_list::at: index out of range");
  return _node(index_key(index));
}

const state &state_list::at(std::size_t index) const {
  if (index >= size())
    throw std::out_of_range("state_list::at: index out of range");
  // const_cast is safe — we're returning a const ref and state::operator()
  // is non-const because it may create children, but here the child exists.
  return const_cast<state &>(_node)(index_key(index));
}

void state_list::pop_back() {
  auto n = size();
  if (n == 0)
    throw std::out_of_range("state_list::pop_back: list is empty");
  // Remove the last child by resetting it.
  _node(index_key(n - 1)).reset(true, false);
  set_size(n - 1);
}

void state_list::erase(std::size_t index) {
  auto n = size();
  if (index >= n)
    throw std::out_of_range("state_list::erase: index out of range");

  // Shift children [index+1, n) down by one.
  for (std::size_t i = index; i + 1 < n; ++i)
    copy_element(_node(index_key(i)), _node(index_key(i + 1)));

  // Remove the last (now-duplicated) element.
  _node(index_key(n - 1)).reset(true, false);
  set_size(n - 1);
}

state &state_list::insert(std::size_t index) {
  auto n = size();
  if (index > n)
    throw std::out_of_range("state_list::insert: index out of range");

  // Shift children [index, n) up by one, starting from the end.
  for (std::size_t i = n; i > index; --i)
    copy_element(_node(index_key(i)), _node(index_key(i - 1)));

  // Initialize the new element.
  auto &inserted = _node(index_key(index));
  inserted.reset(true, false);
  set_size(n + 1);
  return inserted;
}

void state_list::clear() {
  auto n = size();
  for (std::size_t i = 0; i < n; ++i)
    _node(index_key(i)).reset(true, false);
  set_size(0);
}

} // namespace cvc
