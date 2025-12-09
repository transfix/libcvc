# State Futures API

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [API Methods](#api-methods)
  - [Value Methods](#value-methods)
  - [Data Methods](#data-methods)
- [Usage Examples](#usage-examples)
  - [Producer-Consumer Pattern](#producer-consumer-pattern)
  - [Request-Response Pattern](#request-response-pattern)
  - [Value Monitoring with Callbacks](#value-monitoring-with-callbacks)
  - [Multiple Consumers](#multiple-consumers)
  - [Pipeline Pattern](#pipeline-pattern)
- [Implementation Details](#implementation-details)
  - [Synchronization](#synchronization)
  - [Exception Safety](#exception-safety)
  - [Performance Considerations](#performance-considerations)
- [Thread Safety](#thread-safety)
- [Comparison with Traditional Approaches](#comparison-with-traditional-approaches)
- [Testing](#testing)

## Overview

The `cvc::state` futures API provides async/await-style functionality for state value retrieval. This enables one thread to block and wait for another thread to set a value, implementing producer-consumer patterns and async message passing through the state tree.

## Features

- **Blocking waits**: Wait indefinitely for a value to be set
- **Timeout support**: Wait with configurable timeout
- **Callbacks**: Register callbacks that fire when values change
- **Type-safe**: Fully templated with type conversion
- **Thread-safe**: All operations use proper synchronization

## API Methods

### Value Methods

#### `wait_for_value<T>(timeout)`
Blocks until a value is set, with optional timeout.

```cpp
// Wait indefinitely
int value = cvcstate("my.key").wait_for_value<int>();

// Wait with timeout (throws on timeout)
double value = cvcstate("my.key").wait_for_value<double>(
    boost::chrono::milliseconds(1000)
);
```

#### `value<T>(callback)`
Get current value and optionally register a callback for changes.

```cpp
// Get value with callback
int val = cvcstate("my.key").value<int>([](int newValue) {
    std::cout << "Value changed to: " << newValue << std::endl;
});
```

#### `value_future<T>()`
Returns a `state_future<T>` object for advanced control.

```cpp
auto future = cvcstate("my.key").value_future<std::string>();

// Check if ready (non-blocking)
if (future.is_ready()) {
    std::string val = future.get();
}

// Wait with timeout
if (future.wait_for(boost::chrono::seconds(5))) {
    std::string val = future.get();
}

// Get with timeout (throws on timeout)
std::string val = future.get_for(boost::chrono::seconds(5));

// Block indefinitely
std::string val = future.get();
```

### Data Methods

#### `wait_for_data<T>(timeout)`
Blocks until data is set, with optional timeout.

```cpp
// Wait indefinitely
MyStruct data = cvcstate("my.key").wait_for_data<MyStruct>();

// Wait with timeout
MyStruct data = cvcstate("my.key").wait_for_data<MyStruct>(
    boost::chrono::milliseconds(500)
);
```

#### `data<T>(callback)`
Get current data and optionally register a callback for changes.

```cpp
MyStruct current = cvcstate("my.key").data<MyStruct>([](MyStruct newData) {
    // Called whenever data changes
    processData(newData);
});
```

## Usage Examples

### Producer-Consumer Pattern

```cpp
// Consumer thread waits for producer
boost::thread consumer([]() {
    // Blocks until producer sets the value
    int result = cvcstate("work.result").wait_for_value<int>();
    std::cout << "Got result: " << result << std::endl;
});

// Producer thread does work and sets result
boost::thread producer([]() {
    int result = do_expensive_computation();
    cvcstate("work.result").value(result);
});

consumer.join();
producer.join();
```

### Request-Response Pattern

```cpp
// Server thread waiting for requests
void serverThread() {
    while (running) {
        // Wait for request (with timeout to check running flag)
        try {
            std::string request = cvcstate("server.request")
                .wait_for_value<std::string>(boost::chrono::seconds(1));
            
            // Process and send response
            std::string response = handleRequest(request);
            cvcstate("server.response").value(response);
            
        } catch (std::runtime_error&) {
            // Timeout - check if we should keep running
        }
    }
}

// Client thread making requests
std::string makeRequest(const std::string& req) {
    cvcstate("server.request").value(req);
    return cvcstate("server.response")
        .wait_for_value<std::string>(boost::chrono::seconds(10));
}
```

### Value Monitoring with Callbacks

```cpp
// Set up monitoring
std::atomic<int> updateCount(0);

int currentValue = cvcstate("sensor.temperature").value<double>([&](double temp) {
    updateCount++;
    if (temp > 100.0) {
        std::cerr << "WARNING: Temperature too high: " << temp << std::endl;
    }
});

// The callback will fire every time the temperature value changes
// This happens asynchronously via boost::signals2
```

### Multiple Consumers

```cpp
// Multiple threads can wait on the same state value
std::vector<boost::thread> consumers;

for (int i = 0; i < 5; ++i) {
    consumers.emplace_back([i]() {
        // All threads will be notified when value is set
        int value = cvcstate("broadcast.value").wait_for_value<int>();
        std::cout << "Consumer " << i << " got: " << value << std::endl;
    });
}

// Producer sets value once - all consumers wake up
boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
cvcstate("broadcast.value").value(42);

for (auto& t : consumers) {
    t.join();
}
```

### Pipeline Pattern

```cpp
// Stage 1: Data acquisition
boost::thread stage1([]() {
    auto data = acquireData();
    cvcstate("pipeline.stage1.output").data(data);
});

// Stage 2: Processing (waits for stage 1)
boost::thread stage2([]() {
    auto input = cvcstate("pipeline.stage1.output")
        .wait_for_data<RawData>();
    auto processed = processData(input);
    cvcstate("pipeline.stage2.output").data(processed);
});

// Stage 3: Output (waits for stage 2)
boost::thread stage3([]() {
    auto result = cvcstate("pipeline.stage2.output")
        .wait_for_data<ProcessedData>();
    outputResults(result);
});

stage1.join();
stage2.join();
stage3.join();
```

## Implementation Details

### Synchronization

- Uses `boost::condition_variable` for blocking waits
- Uses `boost::signals2` for callbacks
- All operations are protected by mutex locks
- Supports multiple waiters on the same state node

### Exception Safety

- Timeout operations throw `std::runtime_error` with descriptive messages
- Type conversion errors in callbacks are silently caught
- Move semantics for `state_future` prevent accidental copies

### Performance Considerations

- Callbacks execute in the thread that modifies the state
- Futures wake all waiting threads when value changes
- Minimal overhead when features are not used
- Condition variables efficiently wait without busy-spinning

## Thread Safety

All futures API methods are thread-safe and can be called concurrently from multiple threads. The implementation uses:

- Mutex protection for all state modifications
- Condition variables for efficient blocking
- Signal/slot mechanism for callbacks
- Atomic operations where appropriate

## Comparison with Traditional Approaches

### Before (polling):
```cpp
// Inefficient busy-wait polling
while (!cvcstate("key").initialized()) {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
}
int val = cvcstate("key").value<int>();
```

### After (futures):
```cpp
// Efficient blocking wait
int val = cvcstate("key").wait_for_value<int>();
```

## Testing

The futures API includes comprehensive tests covering:
- Blocking waits (indefinite and with timeout)
- Callbacks for value and data changes  
- Future objects with timeout support
- Producer-consumer patterns
- Multiple waiters on same state
- Race conditions and thread safety

All 11 futures tests pass, exercising the API under various concurrent scenarios.
