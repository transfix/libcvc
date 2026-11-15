;; The second test guest of ariadne_wasm_browser_test / ariadne_wasm_transform_test, transformed for
;; the browser beside guest.wat (cvc_ariadne_wasm_test_guests): a state import that pauses the guest
;; and resumes it in the same slice, a memory.grow loop, recursions that pause deep in the stack (at
;; a yield and at gas checks), and a volume read and sampled by handle into the guest's memory. The
;; ABI is guest.wat's (ARIADNE_SCRIPTING_HOSTS.md §8.4, §8.4.1).
(module
  (import "ari" "reply" (func $reply (param i32)))
  (import "ari" "fail" (func $fail (param i32 i32)))
  (import "ari" "yield" (func $yield))
  (import "ari" "state_get" (func $get (param i32 i32) (result i64)))
  (import "ari" "volume_read"
    (func $vread (param i64 i32 i32 i32 i32 i32 i32 i32 i32 i32) (result i64)))
  (import "ari" "volume_sample" (func $vsample (param i64 i32 i32 i32 i32) (result i64)))
  (memory (export "memory") 1)
  (global $heap (mut i32) (i32.const 4096))
  (func $alloc (export "ari_alloc") (param $n i32) (result i32) (local $p i32)
    (local.set $p (global.get $heap))
    (global.set $heap (i32.add (global.get $heap) (local.get $n)))
    (local.get $p))
  (func $pack (param $p i32) (param $n i32) (result i64)
    (i64.or (i64.shl (i64.extend_i32_u (local.get $p)) (i64.const 32))
            (i64.extend_i32_u (local.get $n))))
  ;; A state import's result: its CBOR, or fail with its error's text when it is < 0.
  (func $ok (param $r i64) (result i64) (local $dst i32)
    (local.set $dst (global.get $heap))
    (call $reply (local.get $dst))
    (if (i64.lt_s (local.get $r) (i64.const 0))
      (then (call $fail (local.get $dst) (i32.wrap_i64 (i64.sub (i64.const 0) (local.get $r))))
            (unreachable)))
    (global.set $heap (i32.add (local.get $dst) (i32.wrap_i64 (local.get $r))))
    (call $pack (local.get $dst) (i32.wrap_i64 (local.get $r))))
  (data (i32.const 16) "n")
  ;; (get-n): (state-get "n") through the state import.
  (func (export "get-n") (param i32 i32) (result i64)
    (call $ok (call $get (i32.const 16) (i32.const 1))))
  ;; (get-many): (state-get "n") 200 times; the last.
  (func (export "get-many") (param i32 i32) (result i64) (local $i i32) (local $r i64)
    (loop $l
      (local.set $r (call $ok (call $get (i32.const 16) (i32.const 1))))
      (local.set $i (i32.add (local.get $i) (i32.const 1)))
      (br_if $l (i32.lt_u (local.get $i) (i32.const 200))))
    (local.get $r))
  ;; (hog): grow the memory a page at a time until a grow is refused, then give up.
  (func (export "hog") (param i32 i32) (result i64)
    (loop $l (br_if $l (i32.ne (memory.grow (i32.const 1)) (i32.const -1))))
    (unreachable))
  ;; $n frames down, a yield at the bottom, and one more on the way back up each: $n + 1.
  (func $down (param $n i32) (result i32)
    (if (i32.eqz (local.get $n))
      (then (call $yield) (return (i32.const 1))))
    (i32.add (call $down (i32.sub (local.get $n) (i32.const 1))) (i32.const 1)))
  ;; $n frames down, each looping 20000 times on the way (gas checks every iteration): $n + 1.
  (func $climb (param $n i32) (result i32) (local $i i32)
    (loop $l
      (local.set $i (i32.add (local.get $i) (i32.const 1)))
      (br_if $l (i32.lt_u (local.get $i) (i32.const 20000))))
    (if (i32.eqz (local.get $n))
      (then (return (i32.const 1))))
    (i32.add (call $climb (i32.sub (local.get $n) (i32.const 1))) (i32.const 1)))
  ;; A small count as its CBOR (an unsigned below 24 is its own byte).
  (func $small (param $v i32) (result i64) (local $p i32)
    (local.set $p (call $alloc (i32.const 1)))
    (i32.store8 (local.get $p) (local.get $v))
    (call $pack (local.get $p) (i32.const 1)))
  ;; (deep): 9, from 8 frames down and a yield there.
  (func (export "deep") (param i32 i32) (result i64)
    (call $small (call $down (i32.const 8))))
  ;; (climb): 21, from 20 frames down, sliced wherever the gas runs out.
  (func (export "climb") (param i32 i32) (result i64)
    (call $small (call $climb (i32.const 20))))
  ;; An export's first argument as a handle (its CBOR: the array's head, the tag's five bytes, then
  ;; 0x1b and the handle's eight, big-endian), and the k-th of the small integers after it.
  (func $h (param $p i32) (result i64) (local $i i32) (local $v i64)
    (loop $l
      (local.set $v (i64.or (i64.shl (local.get $v) (i64.const 8))
        (i64.load8_u (i32.add (local.get $p) (i32.add (i32.const 7) (local.get $i))))))
      (local.set $i (i32.add (local.get $i) (i32.const 1)))
      (br_if $l (i32.lt_u (local.get $i) (i32.const 8))))
    (local.get $v))
  (func $arg (param $p i32) (param $k i32) (result i32)
    (i32.load8_u (i32.add (local.get $p) (i32.add (i32.const 15) (local.get $k)))))
  ;; A data call's result; fail with its error's text when it is < 0.
  (func $got (param $r i64) (result i32) (local $dst i32)
    (if (i64.lt_s (local.get $r) (i64.const 0))
      (then (local.set $dst (global.get $heap))
            (call $reply (local.get $dst))
            (call $fail (local.get $dst) (i32.wrap_i64 (i64.sub (i64.const 0) (local.get $r))))
            (unreachable)))
    (i32.wrap_i64 (local.get $r)))
  ;; CBOR unsigned n (< 256) at p; the bytes it took.
  (func $uint (param $p i32) (param $n i32) (result i32)
    (if (i32.lt_u (local.get $n) (i32.const 24))
      (then (i32.store8 (local.get $p) (local.get $n)) (return (i32.const 1))))
    (i32.store8 (local.get $p) (i32.const 0x18))
    (i32.store8 offset=1 (local.get $p) (local.get $n))
    (i32.const 2))
  ;; (vread h x y z nx ny nz lod): the voxels (unsigned chars), as a list.
  (func (export "vread") (param $p i32) (param $n i32) (result i64)
    (local $dst i32) (local $r i32) (local $i i32) (local $out i32) (local $at i32)
    (local.set $dst (global.get $heap))
    (local.set $r (call $got (call $vread (call $h (local.get $p))
      (call $arg (local.get $p) (i32.const 0)) (call $arg (local.get $p) (i32.const 1))
      (call $arg (local.get $p) (i32.const 2)) (call $arg (local.get $p) (i32.const 3))
      (call $arg (local.get $p) (i32.const 4)) (call $arg (local.get $p) (i32.const 5))
      (call $arg (local.get $p) (i32.const 6)) (local.get $dst) (i32.const 64))))
    (local.set $out (i32.add (local.get $dst) (local.get $r)))
    (i32.store8 (local.get $out) (i32.or (i32.const 0x80) (local.get $r)))
    (local.set $at (i32.const 1))
    (block $done
      (loop $l
        (br_if $done (i32.ge_u (local.get $i) (local.get $r)))
        (local.set $at (i32.add (local.get $at)
          (call $uint (i32.add (local.get $out) (local.get $at))
                      (i32.load8_u (i32.add (local.get $dst) (local.get $i))))))
        (local.set $i (i32.add (local.get $i) (i32.const 1)))
        (br $l)))
    (global.set $heap (i32.add (local.get $out) (local.get $at)))
    (call $pack (local.get $out) (local.get $at)))
  ;; (sample h): the value at (1, 0, 0) (the f64 point at 64), rounded, as one small count.
  (data (i32.const 64) "\00\00\00\00\00\00\f0\3f\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00")
  (func (export "sample") (param $p i32) (param $n i32) (result i64) (local $dst i32)
    (local.set $dst (global.get $heap))
    (drop (call $got (call $vsample (call $h (local.get $p)) (i32.const 64) (i32.const 1)
      (local.get $dst) (i32.const 8))))
    (global.set $heap (i32.add (local.get $dst) (i32.const 8)))
    (call $small (i32.trunc_f64_u (f64.add (f64.load (local.get $dst)) (f64.const 0.5)))))
)
