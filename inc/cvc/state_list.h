#ifndef CVC_STATE_LIST_H
#define CVC_STATE_LIST_H

#include <cstddef>
#include <cvc/state.h>
#include <iterator>
#include <string>

namespace cvc {

/// Ordered-list wrapper over cvc::state children.
///
/// Encodes list elements as zero-padded decimal child names
/// ("000000", "000001", ...) and stores the count in a sibling
/// "__len__" node.  Lexicographic map order equals positional order.
///
/// This is a non-owning view: the underlying state node must outlive
/// the state_list instance.
class state_list {
public:
  /// Default padding width (6 digits → up to 999,999 elements).
  static constexpr std::size_t default_pad_width = 6;

  /// Construct a list view over the given state node.
  /// @param node   The state node whose children represent the list.
  /// @param pad_width  Number of digits for zero-padded keys.
  explicit state_list(state &node, std::size_t pad_width = default_pad_width);

  /// Append a new element and return a reference to it.
  state &push_back();

  /// Access element at the given index.
  /// @throws std::out_of_range if index >= size().
  state &at(std::size_t index);
  const state &at(std::size_t index) const;

  /// Remove the last element.
  /// @throws std::out_of_range if the list is empty.
  void pop_back();

  /// Erase the element at the given index, then reindex subsequent
  /// elements so keys remain contiguous.
  /// @throws std::out_of_range if index >= size().
  void erase(std::size_t index);

  /// Insert a new element at the given index, shifting subsequent
  /// elements up by one.
  /// @throws std::out_of_range if index > size() (past-end insert is allowed).
  /// @returns a reference to the newly created element.
  state &insert(std::size_t index);

  /// Number of elements in the list.
  std::size_t size() const;

  /// Whether the list is empty.
  bool empty() const;

  /// Remove all elements and reset the length.
  void clear();

  /// Access the underlying state node.
  state &node() noexcept { return _node; }
  const state &node() const noexcept { return _node; }

  // --- iterator ----------------------------------------------------------

  class iterator {
  public:
    using iterator_category = std::random_access_iterator_tag;
    using value_type = state;
    using difference_type = std::ptrdiff_t;
    using pointer = state *;
    using reference = state &;

    iterator() = default;
    iterator(state_list *list, std::size_t pos) : _list(list), _pos(pos) {}

    reference operator*() const { return _list->at(_pos); }
    pointer operator->() const { return &_list->at(_pos); }

    iterator &operator++() {
      ++_pos;
      return *this;
    }
    iterator operator++(int) {
      auto tmp = *this;
      ++_pos;
      return tmp;
    }
    iterator &operator--() {
      --_pos;
      return *this;
    }
    iterator operator--(int) {
      auto tmp = *this;
      --_pos;
      return tmp;
    }

    iterator &operator+=(difference_type n) {
      _pos += n;
      return *this;
    }
    iterator &operator-=(difference_type n) {
      _pos -= n;
      return *this;
    }
    iterator operator+(difference_type n) const { return {_list, _pos + n}; }
    iterator operator-(difference_type n) const { return {_list, _pos - n}; }
    difference_type operator-(const iterator &o) const {
      return static_cast<difference_type>(_pos) - static_cast<difference_type>(o._pos);
    }

    reference operator[](difference_type n) const { return _list->at(_pos + n); }

    bool operator==(const iterator &o) const { return _pos == o._pos; }
    bool operator!=(const iterator &o) const { return _pos != o._pos; }
    bool operator<(const iterator &o) const { return _pos < o._pos; }
    bool operator<=(const iterator &o) const { return _pos <= o._pos; }
    bool operator>(const iterator &o) const { return _pos > o._pos; }
    bool operator>=(const iterator &o) const { return _pos >= o._pos; }

  private:
    state_list *_list = nullptr;
    std::size_t _pos = 0;
  };

  iterator begin() { return {this, 0}; }
  iterator end() { return {this, size()}; }

  // --- key utilities -----------------------------------------------------

  /// Convert a positional index to its zero-padded key string.
  std::string index_key(std::size_t i) const;

private:
  state &_node;
  std::size_t _pad_width;

  static constexpr const char *len_key = "__len__";

  void set_size(std::size_t n);
};

} // namespace cvc

#endif // CVC_STATE_LIST_H
