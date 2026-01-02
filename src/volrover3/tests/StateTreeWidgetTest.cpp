#include <gtest/gtest.h>
#include <volrover3/StateTreeWidget.h>
#include <volrover3/AppState.h>
#include <QApplication>
#include <cvc/state.h>

class StateTreeWidgetTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a temporary state tree for testing
        testState = &cvc::state::instance()("test_widget");
        
        // Create some test states
        testState->operator()("child1").value("value1");
        testState->operator()("child2").value("value2");
        testState->operator()("nested")("deep").value("deep_value");
        
        widget = new StateTreeWidget();
        widget->setRootState(testState);
    }
    
    void TearDown() override {
        delete widget;
        // Clean up test states
        testState->reset();
    }
    
    cvc::state* testState;
    StateTreeWidget* widget;
};

// Test that the widget initializes correctly
TEST_F(StateTreeWidgetTest, WidgetInitialization) {
    ASSERT_NE(widget, nullptr);
    EXPECT_TRUE(widget->isVisible() == false); // Not shown by default
}

// Test that initialized states appear in the tree
TEST_F(StateTreeWidgetTest, InitializedStatesAppear) {
    // The widget should show initialized states
    // We can't easily test Qt widget internals without a full GUI test,
    // but we can verify the state structure
    
    auto children = testState->children();
    
    // Should have at least our test children
    bool hasChild1 = false, hasChild2 = false, hasNested = false;
    for (const auto& childPath : children) {
        if (childPath.find("test_widget.child1") != std::string::npos) hasChild1 = true;
        if (childPath.find("test_widget.child2") != std::string::npos) hasChild2 = true;
        if (childPath.find("test_widget.nested") != std::string::npos) hasNested = true;
    }
    
    EXPECT_TRUE(hasChild1);
    EXPECT_TRUE(hasChild2);
    EXPECT_TRUE(hasNested);
}

// Test that uninitialized states don't appear
TEST_F(StateTreeWidgetTest, UninitializedStatesHidden) {
    // Create an uninitialized state
    cvc::state& uninit = testState->operator()("uninitialized");
    
    EXPECT_FALSE(uninit.initialized());
    
    // After setting root state, uninitialized states should be filtered
    widget->setRootState(testState);
    
    // The state exists but is not initialized
    EXPECT_FALSE(uninit.initialized());
}

// Test state reset functionality
TEST_F(StateTreeWidgetTest, StateReset) {
    cvc::state& testChild = testState->operator()("child1");
    
    EXPECT_TRUE(testChild.initialized());
    EXPECT_EQ(testChild.value(), "value1");
    
    // Reset the state
    testChild.reset();
    
    EXPECT_FALSE(testChild.initialized());
    EXPECT_EQ(testChild.value(), "");
}

// Test nested state access
TEST_F(StateTreeWidgetTest, NestedStates) {
    cvc::state& nested = testState->operator()("nested");
    nested.value("nested_value"); // Initialize nested state
    EXPECT_TRUE(nested.initialized());
    
    cvc::state& deep = nested("deep");
    EXPECT_TRUE(deep.initialized());
    EXPECT_EQ(deep.value(), "deep_value");
}

// Test state value modification
TEST_F(StateTreeWidgetTest, StateValueModification) {
    cvc::state& testChild = testState->operator()("child1");
    
    EXPECT_EQ(testChild.value(), "value1");
    
    testChild.value("modified_value");
    
    EXPECT_EQ(testChild.value(), "modified_value");
}

// Test state path validation (simulating what the widget does)
TEST_F(StateTreeWidgetTest, PathValidation) {
    // Valid paths
    QRegularExpression pathRegex("^[a-zA-Z_][a-zA-Z0-9_]*(\\.[a-zA-Z_][a-zA-Z0-9_]*)*$");
    
    EXPECT_TRUE(pathRegex.match("valid_name").hasMatch());
    EXPECT_TRUE(pathRegex.match("test.nested.path").hasMatch());
    EXPECT_TRUE(pathRegex.match("_private").hasMatch());
    EXPECT_TRUE(pathRegex.match("name123").hasMatch());
    EXPECT_TRUE(pathRegex.match("app.config.value_1").hasMatch());
    
    // Invalid paths
    EXPECT_FALSE(pathRegex.match("123invalid").hasMatch());
    EXPECT_FALSE(pathRegex.match("path-with-dash").hasMatch());
    EXPECT_FALSE(pathRegex.match("path with space").hasMatch());
    EXPECT_FALSE(pathRegex.match("path..double").hasMatch());
    EXPECT_FALSE(pathRegex.match(".starts_with_dot").hasMatch());
    EXPECT_FALSE(pathRegex.match("ends_with_dot.").hasMatch());
    EXPECT_FALSE(pathRegex.match("has$pecial").hasMatch());
}

// Test hierarchical structure
TEST_F(StateTreeWidgetTest, HierarchicalStructure) {
    // Create a deeper hierarchy
    testState->operator()("level1")("level2")("level3").value("deep");
    
    cvc::state& level1 = testState->operator()("level1");
    cvc::state& level2 = level1("level2");
    cvc::state& level3 = level2("level3");
    
    EXPECT_TRUE(level1.initialized());
    EXPECT_TRUE(level2.initialized());
    EXPECT_TRUE(level3.initialized());
    
    EXPECT_EQ(level3.value(), "deep");
    EXPECT_EQ(level3.name(), "level3");
    EXPECT_EQ(level3.fullName(), "test_widget.level1.level2.level3");
}

// Test immediate children filtering
TEST_F(StateTreeWidgetTest, ImmediateChildrenFilter) {
    // Create nested structure
    testState->operator()("parent")("child")("grandchild").value("value");
    
    auto allChildren = testState->children();
    
    // allChildren is recursive, so it contains all descendants
    // We need to filter for immediate children only
    std::string parentFullName = testState->fullName();
    std::set<std::string> immediateChildren;
    
    for (const auto& childFullName : allChildren) {
        if (childFullName.find(parentFullName) == 0) {
            std::string relativePath = childFullName.substr(parentFullName.length());
            
            if (!relativePath.empty() && relativePath[0] == '.') {
                relativePath = relativePath.substr(1);
            }
            
            if (!relativePath.empty() && relativePath.find('.') == std::string::npos) {
                immediateChildren.insert(relativePath);
            }
        }
    }
    
    // Should have our immediate children but not grandchildren
    EXPECT_TRUE(immediateChildren.find("child1") != immediateChildren.end() ||
                immediateChildren.find("child2") != immediateChildren.end() ||
                immediateChildren.find("parent") != immediateChildren.end());
    
    // Should NOT have grandchild as immediate child
    EXPECT_TRUE(immediateChildren.find("grandchild") == immediateChildren.end());
}

// Test state data type information
TEST_F(StateTreeWidgetTest, StateDataTypes) {
    cvc::state& intState = testState->operator()("int_value");
    cvc::state& doubleState = testState->operator()("double_value");
    cvc::state& stringState = testState->operator()("string_value");
    
    intState.value(42);
    doubleState.value(3.14);
    stringState.value("text");
    
    EXPECT_EQ(intState.value(), "42");
    // Floating point precision: 3.14 may be stored as 3.1400000000000001
    EXPECT_NE(doubleState.value(), "");
    EXPECT_TRUE(doubleState.value().find("3.14") == 0);
    EXPECT_EQ(stringState.value(), "text");
}

// Test state with data (boost::any)
TEST_F(StateTreeWidgetTest, StateData) {
    cvc::state& dataState = testState->operator()("with_data");
    
    std::shared_ptr<int> testData = std::make_shared<int>(123);
    dataState.data(boost::any(testData));
    dataState.value("has_data");
    
    EXPECT_TRUE(dataState.initialized());
    EXPECT_FALSE(dataState.data().empty());
    
    auto retrieved = boost::any_cast<std::shared_ptr<int>>(dataState.data());
    EXPECT_EQ(*retrieved, 123);
}

// Test state value type name
TEST_F(StateTreeWidgetTest, StateValueTypeName) {
    cvc::state& typedState = testState->operator()("typed");
    
    typedState.value(100);
    
    // After setting an int value, the type name should be set
    std::string typeName = typedState.valueTypeName();
    EXPECT_FALSE(typeName.empty());
}

// Test last modified time
TEST_F(StateTreeWidgetTest, LastModifiedTime) {
    cvc::state& timedState = testState->operator()("timed");
    
    auto before = boost::posix_time::microsec_clock::universal_time();
    timedState.value("test");
    auto after = boost::posix_time::microsec_clock::universal_time();
    
    auto lastMod = timedState.lastMod();
    
    EXPECT_GE(lastMod, before);
    EXPECT_LE(lastMod, after);
}

// Test widget refresh doesn't crash
TEST_F(StateTreeWidgetTest, WidgetRefresh) {
    EXPECT_NO_THROW(widget->refresh());
    
    // Add more states and refresh again
    testState->operator()("new_state").value("new");
    EXPECT_NO_THROW(widget->refresh());
}

int main(int argc, char **argv) {
    // Qt application needed for widget tests
    QApplication app(argc, argv);
    
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
