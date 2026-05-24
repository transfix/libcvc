# state_exec: S-Expression DSL Engine for cvc::state

Date: May 23, 2026

## Overview

`state_exec` is a port of the `txwtf.cfos` Python S-expression DSL to C++20,
built on top of `cvc::state`.  It provides a sandboxed scripting environment
where programs execute as processes managed by a scheduler, with their entire
state (code, stack, environments, statistics) materialized in the state tree
for inspection, serialization, migration, and distributed execution.

Full design: [STATE_EXEC_PORTING_PLAN.md](STATE_EXEC_PORTING_PLAN.md)

## Status

### Phase 1: Foundation ✅ COMPLETE

| Component | Files | Tests | Status |
|---|---|---|---|
| `state_list` | `inc/cvc/state_list.h`, `src/cvc/state_list.cpp` | 20 | ✅ |
| `state_eviction_store` | `inc/cvc/state_eviction_store.h`, `src/cvc/state_eviction_store.cpp` | (in memory_manager tests) | ✅ |
| `state_memory_manager` | `inc/cvc/state_memory_manager.h`, `src/cvc/state_memory_manager.cpp` | 20 | ✅ |
| Core types (`value_t`, `symbol`, `closure`, `environment`) | `inc/cvc/state_exec/types.h`, `src/cvc/state_exec/types.cpp` | 26 | ✅ |
| S-expression parser | `inc/cvc/state_exec/parser.h`, `src/cvc/state_exec/parser.cpp` | 30 | ✅ |

**Total Phase 1 tests: 96**

### Phase 2: Evaluators + Built-ins — COMPLETE ✅

| Component | Status | Tests |
|-----------|--------|-------|
| `builtins` (35+ operators) | ✅ Done | 38 |
| `evaluator` (sync recursive) | ✅ Done | 39 |
| `stackless_evaluator` (sync stackless) | ✅ Done | 31 |
| `state_value_codec` (value/env round-trip) | ✅ Done | 23 |
| Evaluator state serialization | ✅ Done | 4 |
| Cross-evaluator consistency | ✅ Done | 32 |

**Phase 2 test total: 167**

### Phase 3: Async Evaluators — COMPLETE ✅

| Component | Header | Tests |
|---|---|---|
| `task<T>` coroutine type | `task.h` | 6 |
| `async_evaluator` | `async_evaluator.h` | 32 |
| `async_stackless_evaluator` | `async_stackless_evaluator.h` | 18 |
| Cross-evaluator consistency (4 variants) | — | 3 |

**Phase 3 test total: 59**

### Phase 4: Scheduler + Resource Limits — NOT STARTED

- Process abstraction and lifecycle management
- Per-process memory tracking (`memory_tracker`)
- Time limit, message limit, step limit enforcement
- Sync and async schedulers with scheduling policies
- Process forking (§4.16)

### Phase 5: State Integration + Intrinsics — NOT STARTED

- DSL intrinsics for state tree interaction
- Expiry, messaging, observation intrinsics
- Cluster resource policies
- Standard library (`stdlib_registry`)
- Data objects and batched state changes
- State tree ACL (cluster-consensus enforcement)

### Phase 6: Distributed Execution — NOT STARTED

- Scheduling coordination via `state_message_bus`
- Process migration across nodes
- Cross-cluster observation and admin controls

### Phase 7: CMake Integration + Polish — NOT STARTED

- Feature flags (`CVC_STATE_EXEC`, `CVC_STATE_MEMORY_MANAGER`)
- Documentation and benchmarks

## Architecture

```
┌──────────────────────────────────────────────────────────┐
│                    DSL Programs                          │
│  (defun fib (n) (if (<= n 1) n (+ (fib (- n 1)) ...))) │
├──────────────────────────────────────────────────────────┤
│  Parser  →  value_t AST  →  Evaluator  →  value_t      │
├──────────────────────────────────────────────────────────┤
│  Scheduler (process mgmt, resource limits, signals)     │
├──────────────────────────────────────────────────────────┤
│  state_list │ state_memory_manager │ state_value_codec   │
├──────────────────────────────────────────────────────────┤
│  cvc::state (tree) │ state_object<T> (CRTP batching)    │
├──────────────────────────────────────────────────────────┤
│  Distributed infra (cluster shard, message bus, etc.)   │
└──────────────────────────────────────────────────────────┘
```

## Key Design Decisions

- **Evaluator tier model**: Four evaluator implementations exist, but only
  `async_stackless_evaluator` is used by the scheduler for cluster-managed
  processes.  It is async (non-blocking) and stackless (serializable), both
  of which are required for migration, pause/resume, and distributed
  observation.  Programs run through the other three evaluators (`evaluator`,
  `async_evaluator`, `stackless_evaluator`) are local-only — they do not
  appear in `ps-all`, cannot be migrated, and are not first-class system
  citizens.  Developers may use any evaluator for local/embedded scripting.
- **Local sovereignty model**: No local ACL enforcement; cluster consensus
  protects shared state at the replication boundary.
- **Two-tier memory management**: Tree-level LRU eviction + per-process
  byte tracking.
- **Message-based distributed coordination**: All cluster operations flow
  through `state_message_bus`.
- **C++20 features**: `std::variant`, `std::span`, coroutines, `<=>`,
  structured bindings, `std::from_chars`.

## Dependencies

- Boost (property_tree, signals2, thread, chrono, any, lexical_cast)
- Google Test (testing)
- C++20 compiler (GCC 11+, Clang 14+)
- No new external dependencies beyond existing libcvc requirements
