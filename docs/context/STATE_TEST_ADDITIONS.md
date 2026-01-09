# State Test Suite Additions

## Overview
Added comprehensive tests to `src/cvc/tests/state_test.cpp` to cover new `reset()` parameters and `state_object` threading control functionality.

## Test Summary

### Reset Parameter Tests (6 tests)
Tests for the new `reset(bool resetChildren, bool fireCallbacks)` signature:

1. **ResetWithoutChildren** - `reset(false, true)`
   - Tests that resetting without children detaches child states
   - Verifies parent is reset but children are cleared from parent's child map
   - Accessing children via parent path creates new uninitialized states

2. **ResetWithChildren** - `reset(true, true)` 
   - Tests that resetting with children recursively resets entire hierarchy
   - Verifies parent and all descendants (including grandchildren) are reset

3. **ResetWithoutCallbacks** - `reset(true, false)`
   - Tests that reset without callbacks does not fire `valueChanged` signals
   - Uses signal connection to verify callbacks are not invoked

4. **ResetWithCallbacks** - `reset(true, true)`
   - Tests that reset with callbacks fires `valueChanged` signals via `touch()`
   - Verifies callbacks are invoked when `fireCallbacks=true`

5. **ResetParameterCombinations**
   - Comprehensive test of all 4 parameter combinations:
     * `reset(false, false)` - detach children, no callbacks
     * `reset(false, true)` - detach children, fire callbacks
     * `reset(true, false)` - reset children, no callbacks  
     * `reset(true, true)` - reset children, fire callbacks (default)

### Threading Control Tests (6 tests)
Tests for `state_object<T>::setUseThreading()` and `getUseThreading()`:

1. **StateObjectThreadingDisabled**
   - Tests that `setUseThreading(false)` causes synchronous execution
   - Verifies `handleStateChanged()` runs on same thread as caller
   - Uses thread ID comparison to validate synchronous behavior

2. **StateObjectThreadingEnabled**
   - Tests that `setUseThreading(true)` enables threaded execution (default)
   - Verifies `handleStateChanged()` runs on different thread
   - Validates asynchronous callback behavior

3. **StateObjectThreadingToggle**
   - Tests dynamic switching between threaded and synchronous modes
   - Starts with threading disabled, then enables it mid-test
   - Verifies mode changes take effect immediately

4. **StateObjectBatchingWithThreadingDisabled**
   - Tests batching behavior with threading disabled
   - Verifies `startBatch()`/`endBatch()` work synchronously
   - Confirms batched changes execute on calling thread

5. **StateObjectThreadingPerTemplateType**
   - Tests that threading control is per-template-instantiation
   - Creates two different `state_object<T>` types (TypeA, TypeB)
   - Verifies setting threading for TypeA doesn't affect TypeB
   - Validates template-level isolation of threading flag

### Helper Class
**ThreadingTestObject**
- Inherits from `state_object<ThreadingTestObject>`
- Tracks execution context (synchronous vs threaded)
- Counts callbacks and identifies which thread they run on
- Used across threading tests to validate behavior

## Key Implementation Details

### Reset Behavior
- `reset(resetChildren, fireCallbacks)` - both parameters default to `true`
- When `resetChildren=false`, calls `_children.clear()` to detach children
- When `fireCallbacks=true`, calls `touch()` which fires `valueChanged()`, `dataChanged()`, and parent's `childChanged()`
- Detached children persist as `shared_ptr` but are no longer accessible via parent

### Threading Control
- Static `_useThreading` flag per template instantiation
- `setUseThreading(bool)` - enables/disables threading for all instances of `state_object<T>`
- `getUseThreading()` - queries current threading setting
- When disabled: `handleStateChanged()` runs synchronously on calling thread
- When enabled: `handleStateChanged()` spawns thread (production default)
- Affects both `stateChanged()` and `endBatch()` execution paths

## Test Results
All tests pass:
- 149 state tests passed (1 skipped stress test)
- 4 volrover3 tests passed (AppStateTest, GraphicsNodeTest, VolumeNodeTest, NullGraphicNodeTest)

## Signal Usage Pattern
Tests use `.valueChanged.connect()` signal pattern:
```cpp
auto conn = cvcstate("path.to.state").valueChanged.connect([&callback]() {
  // Handle value change
});
```

## Benefits
1. **Documentation**: Tests serve as executable documentation for new features
2. **Regression Prevention**: Catches breaking changes to reset() or threading behavior
3. **Validation**: Confirms threading control works as designed
4. **Coverage**: Tests all parameter combinations and edge cases
