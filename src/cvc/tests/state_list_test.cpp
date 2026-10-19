#include <cvc/core/app.h>
#include <cvc/state/state_list.h>
#include <gtest/gtest.h>
#include <stdexcept>

class StateListTest : public ::testing::Test {
protected:
  void SetUp() override {
    app_ = std::make_unique<cvc::app>();
    root_ = &cvc::state::instance(*app_)("test_list");
  }

  std::unique_ptr<cvc::app> app_;
  cvc::state *root_ = nullptr;
};

TEST_F(StateListTest, EmptyList) {
  cvc::state_list list(*root_);
  EXPECT_EQ(list.size(), 0u);
  EXPECT_TRUE(list.empty());
}

TEST_F(StateListTest, PushBackAndAt) {
  cvc::state_list list(*root_);

  auto &first = list.push_back();
  first.value("hello");
  EXPECT_EQ(list.size(), 1u);
  EXPECT_FALSE(list.empty());
  EXPECT_EQ(list.at(0).value(), "hello");

  auto &second = list.push_back();
  second.value("world");
  EXPECT_EQ(list.size(), 2u);
  EXPECT_EQ(list.at(0).value(), "hello");
  EXPECT_EQ(list.at(1).value(), "world");
}

TEST_F(StateListTest, AtOutOfRange) {
  cvc::state_list list(*root_);
  EXPECT_THROW(list.at(0), std::out_of_range);

  list.push_back().value("a");
  EXPECT_THROW(list.at(1), std::out_of_range);
  EXPECT_NO_THROW(list.at(0));
}

TEST_F(StateListTest, PopBack) {
  cvc::state_list list(*root_);
  list.push_back().value("a");
  list.push_back().value("b");
  list.push_back().value("c");

  list.pop_back();
  EXPECT_EQ(list.size(), 2u);
  EXPECT_EQ(list.at(0).value(), "a");
  EXPECT_EQ(list.at(1).value(), "b");
}

TEST_F(StateListTest, PopBackEmpty) {
  cvc::state_list list(*root_);
  EXPECT_THROW(list.pop_back(), std::out_of_range);
}

TEST_F(StateListTest, Erase) {
  cvc::state_list list(*root_);
  list.push_back().value("a");
  list.push_back().value("b");
  list.push_back().value("c");
  list.push_back().value("d");

  // Erase "b" at index 1
  list.erase(1);
  EXPECT_EQ(list.size(), 3u);
  EXPECT_EQ(list.at(0).value(), "a");
  EXPECT_EQ(list.at(1).value(), "c");
  EXPECT_EQ(list.at(2).value(), "d");
}

TEST_F(StateListTest, EraseFirst) {
  cvc::state_list list(*root_);
  list.push_back().value("a");
  list.push_back().value("b");

  list.erase(0);
  EXPECT_EQ(list.size(), 1u);
  EXPECT_EQ(list.at(0).value(), "b");
}

TEST_F(StateListTest, EraseLast) {
  cvc::state_list list(*root_);
  list.push_back().value("a");
  list.push_back().value("b");

  list.erase(1);
  EXPECT_EQ(list.size(), 1u);
  EXPECT_EQ(list.at(0).value(), "a");
}

TEST_F(StateListTest, EraseOutOfRange) {
  cvc::state_list list(*root_);
  list.push_back().value("x");
  EXPECT_THROW(list.erase(1), std::out_of_range);
}

TEST_F(StateListTest, Insert) {
  cvc::state_list list(*root_);
  list.push_back().value("a");
  list.push_back().value("c");

  // Insert "b" at index 1
  list.insert(1).value("b");
  EXPECT_EQ(list.size(), 3u);
  EXPECT_EQ(list.at(0).value(), "a");
  EXPECT_EQ(list.at(1).value(), "b");
  EXPECT_EQ(list.at(2).value(), "c");
}

TEST_F(StateListTest, InsertAtBeginning) {
  cvc::state_list list(*root_);
  list.push_back().value("b");

  list.insert(0).value("a");
  EXPECT_EQ(list.size(), 2u);
  EXPECT_EQ(list.at(0).value(), "a");
  EXPECT_EQ(list.at(1).value(), "b");
}

TEST_F(StateListTest, InsertAtEnd) {
  cvc::state_list list(*root_);
  list.push_back().value("a");

  list.insert(1).value("b");
  EXPECT_EQ(list.size(), 2u);
  EXPECT_EQ(list.at(0).value(), "a");
  EXPECT_EQ(list.at(1).value(), "b");
}

TEST_F(StateListTest, InsertOutOfRange) {
  cvc::state_list list(*root_);
  EXPECT_THROW(list.insert(1), std::out_of_range);
  EXPECT_NO_THROW(list.insert(0)); // Past-end insert into empty list is OK
}

TEST_F(StateListTest, Clear) {
  cvc::state_list list(*root_);
  list.push_back().value("a");
  list.push_back().value("b");
  list.push_back().value("c");

  list.clear();
  EXPECT_EQ(list.size(), 0u);
  EXPECT_TRUE(list.empty());
}

TEST_F(StateListTest, Iterator) {
  cvc::state_list list(*root_);
  list.push_back().value("x");
  list.push_back().value("y");
  list.push_back().value("z");

  std::vector<std::string> values;
  for (auto it = list.begin(); it != list.end(); ++it) {
    values.push_back(it->value());
  }
  ASSERT_EQ(values.size(), 3u);
  EXPECT_EQ(values[0], "x");
  EXPECT_EQ(values[1], "y");
  EXPECT_EQ(values[2], "z");
}

TEST_F(StateListTest, IteratorArithmetic) {
  cvc::state_list list(*root_);
  list.push_back().value("a");
  list.push_back().value("b");
  list.push_back().value("c");

  auto it = list.begin();
  EXPECT_EQ((*(it + 2)).value(), "c");
  EXPECT_EQ(it[1].value(), "b");
  EXPECT_EQ(list.end() - list.begin(), 3);
}

TEST_F(StateListTest, IndexKeyFormat) {
  cvc::state_list list(*root_);
  EXPECT_EQ(list.index_key(0), "000000");
  EXPECT_EQ(list.index_key(1), "000001");
  EXPECT_EQ(list.index_key(42), "000042");
  EXPECT_EQ(list.index_key(999999), "999999");
}

TEST_F(StateListTest, CustomPadWidth) {
  cvc::state_list list(*root_, 3);
  EXPECT_EQ(list.index_key(0), "000");
  EXPECT_EQ(list.index_key(42), "042");

  list.push_back().value("first");
  EXPECT_EQ(list.size(), 1u);
  EXPECT_EQ(list.at(0).value(), "first");
}

TEST_F(StateListTest, RangeBasedFor) {
  cvc::state_list list(*root_);
  list.push_back().value("1");
  list.push_back().value("2");
  list.push_back().value("3");

  int count = 0;
  for (auto &elem : list) {
    ++count;
    EXPECT_FALSE(elem.value().empty());
  }
  EXPECT_EQ(count, 3);
}

TEST_F(StateListTest, ChildSubtrees) {
  // Elements can themselves be subtrees, not just flat values.
  cvc::state_list list(*root_);
  auto &elem = list.push_back();
  elem("name").value("Alice");
  elem("age").value("30");

  EXPECT_EQ(list.at(0)("name").value(), "Alice");
  EXPECT_EQ(list.at(0)("age").value(), "30");
}

namespace {
// Three elements with different subtrees, and data() on the last.
void fill_with_subtrees(cvc::state_list &list) {
  auto &a = list.push_back();
  a.value("a");
  a("name").value("A");
  auto &b = list.push_back();
  b.value("b");
  b("name").value("B");
  b("extra.deep").value("E");
  auto &c = list.push_back();
  c.value("c");
  c("other").value("C");
  c.data(42);
}

// The value at `path` under `elem`, "" where there is no node. Creates nothing.
std::string value_at(cvc::state &elem, const std::string &path) {
  cvc::state *n = elem.findDescendant(path);
  return n ? n->value() : std::string();
}
} // namespace

TEST_F(StateListTest, EraseShiftsSubtreesAndData) {
  // Each later element moves down one index with its whole subtree and its data(); what only the
  // element it lands on had is cleared. Nothing is written anywhere else.
  cvc::state_list list(*root_);
  fill_with_subtrees(list);
  list.erase(0);
  ASSERT_EQ(list.size(), 2u);

  cvc::state &e0 = list.at(0);
  EXPECT_EQ(e0.value(), "b");
  EXPECT_EQ(value_at(e0, "name"), "B");
  EXPECT_EQ(value_at(e0, "extra.deep"), "E");
  EXPECT_TRUE(e0.data().empty());

  cvc::state &e1 = list.at(1);
  EXPECT_EQ(e1.value(), "c");
  EXPECT_EQ(value_at(e1, "other"), "C");
  EXPECT_EQ(e1.data<int>(), 42);
  EXPECT_EQ(value_at(e1, "name"), "");
  EXPECT_EQ(value_at(e1, "extra.deep"), "");

  // The copy used to land under each element's own path from the root.
  EXPECT_EQ(e0.findDescendant("test_list"), nullptr);
  EXPECT_EQ(e1.findDescendant("test_list"), nullptr);
}

TEST_F(StateListTest, InsertShiftsSubtreesAndData) {
  cvc::state_list list(*root_);
  fill_with_subtrees(list);
  list.insert(0).value("new");
  ASSERT_EQ(list.size(), 4u);

  cvc::state &e0 = list.at(0);
  EXPECT_EQ(e0.value(), "new");
  EXPECT_EQ(value_at(e0, "name"), "");

  cvc::state &e1 = list.at(1);
  EXPECT_EQ(e1.value(), "a");
  EXPECT_EQ(value_at(e1, "name"), "A");
  EXPECT_EQ(value_at(e1, "extra.deep"), "");

  cvc::state &e2 = list.at(2);
  EXPECT_EQ(e2.value(), "b");
  EXPECT_EQ(value_at(e2, "name"), "B");
  EXPECT_EQ(value_at(e2, "extra.deep"), "E");
  EXPECT_EQ(value_at(e2, "other"), "");
  EXPECT_TRUE(e2.data().empty());

  cvc::state &e3 = list.at(3);
  EXPECT_EQ(e3.value(), "c");
  EXPECT_EQ(value_at(e3, "other"), "C");
  EXPECT_EQ(e3.data<int>(), 42);

  for (std::size_t i = 0; i < list.size(); ++i)
    EXPECT_EQ(list.at(i).findDescendant("test_list"), nullptr) << i;
}

TEST_F(StateListTest, ShiftingKeepsTheNodesAndSignals) {
  // Shifting writes into the existing nodes, so a reference into an element stays valid and its
  // observers hear each change, including a node being cleared because the element moving in
  // lacks it.
  cvc::state_list list(*root_);
  fill_with_subtrees(list);
  cvc::state &name1 = list.at(1)("name");
  int heard = 0;
  auto c = name1.valueChanged.connect([&] { ++heard; });
  list.erase(0); // element 1 now holds what element 2 had, which has no "name"
  EXPECT_EQ(list.at(1).findDescendant("name"), &name1);
  EXPECT_EQ(name1.value(), "");
  EXPECT_EQ(heard, 1);
  list.insert(0); // element 1 gets what element 0 had: name "B"
  EXPECT_EQ(name1.value(), "B");
  EXPECT_EQ(heard, 2);
  c.disconnect();
}
