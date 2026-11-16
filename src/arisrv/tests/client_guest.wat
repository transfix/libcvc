;; The guest a client sends in ctest arisrvx_room_call_guest (client_guests.ari carries it as bytes,
;; assembled from this file: wasm-opt client_guest.wat -o client_guest.wasm). The guest ABI is
;; src/ariadne_wasm/wasm_engine.h's: values cross as CBOR, and an export takes (ptr, len) of its
;; argument list and returns its result's (ptr << 32 | len).
(module
  (import "ari" "call" (func $call (param i32 i32 i32 i32) (result i64)))
  (import "ari" "reply" (func $reply (param i32)))
  (import "ari" "fail" (func $fail (param i32 i32)))
  (import "ari" "state_set" (func $state_set (param i32 i32 i32 i32) (result i64)))
  (import "ari" "state_get" (func $state_get (param i32 i32) (result i64)))
  (memory (export "memory") 1)
  (global $heap (mut i32) (i32.const 1024))
  (func (export "ari_alloc") (param $n i32) (result i32) (local $p i32)
    (local.set $p (global.get $heap))
    (global.set $heap (i32.add (global.get $heap) (local.get $n)))
    (local.get $p))
  (func $pack (param $p i32) (param $n i32) (result i64)
    (i64.or (i64.shl (i64.extend_i32_u (local.get $p)) (i64.const 32))
            (i64.extend_i32_u (local.get $n))))
  ;; The last host call's answer: its result's CBOR, or a failure with its error's text.
  (func $answer (param $r i64) (result i64) (local $dst i32)
    (local.set $dst (global.get $heap))
    (call $reply (local.get $dst))
    (if (i64.lt_s (local.get $r) (i64.const 0))
      (then (call $fail (local.get $dst) (i32.wrap_i64 (i64.sub (i64.const 0) (local.get $r))))
            (unreachable)))
    (global.set $heap (i32.add (global.get $heap) (i32.wrap_i64 (local.get $r))))
    (call $pack (local.get $dst) (i32.wrap_i64 (local.get $r))))
  (data (i32.const 16) "score")
  (data (i32.const 32) "\18\2a") ;; 42
  (data (i32.const 48) "pose")
  (data (i32.const 64) "spawn")
  (data (i32.const 80) "\80") ;; []
  ;; Its arguments, as they came.
  (func (export "run") (param $p i32) (param $n i32) (result i64)
    (call $pack (local.get $p) (local.get $n)))
  ;; (state-set "score" 42), then (state-get "score"): in the client's own participant.
  (func (export "write") (param i32 i32) (result i64)
    (drop (call $answer (call $state_set (i32.const 16) (i32.const 5) (i32.const 32) (i32.const 2))))
    (call $answer (call $state_get (i32.const 16) (i32.const 5))))
  ;; (state-set "pose" 42): a key the relay owns.
  (func (export "pose") (param i32 i32) (result i64)
    (call $answer (call $state_set (i32.const 48) (i32.const 4) (i32.const 32) (i32.const 2))))
  ;; (spawn): not in a client's environment.
  (func (export "spawn") (param i32 i32) (result i64)
    (call $answer (call $call (i32.const 64) (i32.const 5) (i32.const 80) (i32.const 1))))
  ;; Never returns: the step cap stops it.
  (func (export "spin") (param i32 i32) (result i64)
    (loop $l (br $l))
    (i64.const 0)))
