#include <cvc/state_list.h>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace cvc {

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
  for (std::size_t i = index; i + 1 < n; ++i) {
    auto &dst = _node(index_key(i));
    auto &src = _node(index_key(i + 1));
    // Copy value and data from src to dst.
    dst.value(src.value());
    if (src.isData<boost::any>())
      dst.data(src.data());
    // Copy the ptree representation for any child subtree.
    dst.ptree(src.ptree());
  }

  // Remove the last (now-duplicated) element.
  _node(index_key(n - 1)).reset(true, false);
  set_size(n - 1);
}

state &state_list::insert(std::size_t index) {
  auto n = size();
  if (index > n)
    throw std::out_of_range("state_list::insert: index out of range");

  // Shift children [index, n) up by one, starting from the end.
  for (std::size_t i = n; i > index; --i) {
    auto &dst = _node(index_key(i));
    auto &src = _node(index_key(i - 1));
    dst.value(src.value());
    if (src.isData<boost::any>())
      dst.data(src.data());
    dst.ptree(src.ptree());
  }

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
