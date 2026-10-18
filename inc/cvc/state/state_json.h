/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A state subtree's JSON document (state::json(), state::save()) and loading one
// (state::json(j), state::restore()), as walks that can stop after any number of nodes and go
// on later. cvc::state runs them to completion; state_exec's state-json and state-from-json run
// them a slice per scheduler step. One implementation, so the two cannot drift.
//
// The document is one level: an entry per node, keyed by its path relative to the node it
// describes ("" for that node), in pre-order with children in name order, holding the node's
// value, or "" for a node below it with neither a value nor children. Values only, not data().
// It is written exactly as boost::property_tree::write_json() writes that one-level tree (the
// format json() had), and read as read_json() followed by ptree(pt) would read it.

#ifndef CVC_STATE_STATE_JSON_H
#define CVC_STATE_STATE_JSON_H

#include <cstddef>
#include <cvc/state/state.h>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace cvc {

// Pre-order over a subtree, a node per next(). A heap stack of levels, not a call per level.
// Each node's value and children are snapshotted under its own lock, released before going
// down, and the children are held by owning pointer, so a node removed from the tree between two
// calls is still visited (as it was when its parent was snapshotted) and none dangles. Changes
// made between calls show in the nodes visited after them.
class state_subtree_cursor {
public:
  // Walk `start`'s subtree. The cursor keeps `start` alive while it walks.
  explicit state_subtree_cursor(state &start);

  enum class visit {
    listed,   // the node belongs in the document: it has a value, or it is childless below start
    unlisted, // it does not: an empty node with children, or an empty start
    done      // no nodes are left
  };

  // Visit the next node: its path relative to start ("" for start) and its value.
  visit next(std::string &path, std::string &value);

  // Nodes visited so far.
  std::size_t visited() const { return visited_; }

private:
  struct level {
    std::vector<std::pair<state::state_ptr, std::string>> kids; // with their relative paths
    std::size_t next = 0;
  };
  // Snapshot `n`'s value and children, whose paths are `rel`'s plus their names.
  level open(state &n, const std::string &rel, std::string &value);

  state::state_ptr keep_;
  state *start_;
  bool started_ = false;
  std::vector<level> stack_;
  std::size_t visited_ = 0;
};

// The document of a subtree, written a slice at a time.
class state_json_writer {
public:
  explicit state_json_writer(state &start) : cursor_(start) {}

  // Visit at most `nodes` more nodes (0: all that remain). True once the document is complete.
  bool step(std::size_t nodes);

  std::size_t nodes() const { return cursor_.visited(); }
  // The text so far (all of it once step() has returned true).
  const std::string &text() const { return out_; }
  std::string take() { return std::move(out_); }

private:
  state_subtree_cursor cursor_;
  std::string out_ = "{\n";
  std::string path_, value_;
  bool any_ = false, done_ = false;
};

// Load a document into a node, a slice at a time, as state::json(j) does. Every string, number,
// true, false and null at any depth of the document is a value; one that is not empty is set
// at its key relative to the target ("" inside an array: the target itself), in document order.
// Numbers keep their text; true, false and null are those words. Objects and arrays only nest.
// The whole document is checked first, so a malformed one sets nothing; errors are
// boost::property_tree::json_parser_error, as read_json() throws them.
class state_json_reader {
public:
  // `doc` is borrowed: it must outlive the reader, or be passed again to rebind().
  state_json_reader(state &target, std::string_view doc, std::string filename = {});

  // The same text at a new address (the reader keeps only offsets into it).
  void rebind(std::string_view doc) { doc_ = doc; }
  std::string_view doc() const { return doc_; }

  // Scan at most `values` more values (0: all that remain); each is scanned twice, once to
  // check and once to set. True once the document is loaded.
  bool step(std::size_t values);

  // Values scanned so far (both passes), and bytes.
  std::size_t values() const { return values_; }
  std::size_t bytes() const { return bytes_; }
  // Values set so far.
  std::size_t applied() const { return applied_; }

private:
  enum class want { start, value, key_or_end, value_or_end, key, comma_or_end, end, done };
  struct frame {
    bool array;
    std::string key; // an object's current key
  };

  [[noreturn]] void fail(const char *msg) const;
  bool at_end() const { return pos_ >= doc_.size(); }
  char peek() const { return doc_[pos_]; }
  void advance();
  bool have(char c);
  void expect(char c, const char *msg);
  char need(const char *msg) const;
  void skip_ws();
  void scan_string(std::string &out);
  void scan_escape(std::string &out);
  unsigned scan_hex_quad();
  void scan_scalar(std::string &out);
  void emit(const std::string &text);
  want after_value() const { return stack_.empty() ? want::end : want::comma_or_end; }

  state::state_ptr keep_;
  state *target_;
  std::string_view doc_;
  std::string filename_;
  bool applying_ = false; // second pass
  want want_ = want::start;
  std::vector<frame> stack_;
  std::size_t pos_ = 0, line_ = 1, scanned_ = 0;
  std::size_t values_ = 0, bytes_ = 0, applied_ = 0;
};

} // namespace cvc

#endif // CVC_STATE_STATE_JSON_H
