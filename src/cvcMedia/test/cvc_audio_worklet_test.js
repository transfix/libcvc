// cvc_audio_worklet_test.js — cvcMedia's AudioWorklet processors (web/cvc-audio-worklet.js) in
// node, in a stand-in for the AudioWorkletGlobalScope: the mic's blocks, and the speaker's ring
// (what it plays, its ticks, running dry, a drop's cross-fade, an overflow).
//
//   node src/cvcMedia/test/cvc_audio_worklet_test.js src/cvcMedia/web/cvc-audio-worklet.js
'use strict';
const assert = require('assert');
const fs = require('fs');
const vm = require('vm');

const file = process.argv[2];
if (!file) {
  console.error('usage: cvc_audio_worklet_test.js <cvc-audio-worklet.js>');
  process.exit(2);
}

// The scope: sampleRate, AudioWorkletProcessor with a port, registerProcessor.
const processors = {};
class Port {
  constructor() { this.sent = []; this.onmessage = null; }
  postMessage(m) { this.sent.push(m); }
  deliver(m) { this.onmessage({data: m}); } // the main thread's postMessage, arriving
}
const scope = {
  sampleRate: 48000,
  currentFrame: 0,
  AudioWorkletProcessor: class { constructor() { this.port = new Port(); } },
  registerProcessor: (name, cls) => { processors[name] = cls; },
  Float32Array, Math,
};
vm.createContext(scope);
vm.runInContext(fs.readFileSync(file, 'utf8'), scope, {filename: file});
assert.ok(processors['cvc-capture'] && processors['cvc-playback'], 'both processors register');

const Q = 128;
const quantum = (ch, f) => Array.from({length: ch}, (_, c) => Float32Array.from({length: Q}, (_, k) => f(c, k)));
// process()'s outputs: one output of CH channels. o[0][c] is channel c.
const outputs = ch => [Array.from({length: ch}, () => new Float32Array(Q))];

let passed = 0;
function test(name, fn) {
  fn();
  passed++;
  console.log('ok - ' + name);
}

test('capture: 20 ms blocks, interleaved, in order', () => {
  const p = new processors['cvc-capture']({processorOptions: {channels: 2}});
  let t = 0;
  for (let q = 0; q < 30; q++) { // 3840 frames = 4 blocks of 960
    const base = t;
    assert.strictEqual(p.process([quantum(2, (c, k) => (base + k) * (c ? -1 : 1))]), true);
    t += Q;
  }
  const blocks = p.port.sent.filter(m => m.t === 'cap');
  assert.strictEqual(blocks.length, 4);
  let n = 0;
  for (const b of blocks) {
    assert.strictEqual(b.pcm.length, 960 * 2);
    for (let f = 0; f < 960; f++, n++) {
      assert.strictEqual(b.pcm[2 * f], n);
      assert.strictEqual(b.pcm[2 * f + 1], -n);
    }
  }
});

test('capture: no input, nothing sent; stop ends it', () => {
  const p = new processors['cvc-capture']({processorOptions: {channels: 1}});
  for (let q = 0; q < 20; q++)
    p.process([[]]);
  assert.strictEqual(p.port.sent.length, 0);
  p.port.deliver({t: 'stop'});
  assert.strictEqual(p.process([quantum(1, () => 1)]), false);
});

test('playback: plays the ring in order, a tick each block', () => {
  const p = new processors['cvc-playback']({processorOptions: {channels: 1}});
  const pcm = Float32Array.from({length: 2000}, (_, i) => i);
  p.port.deliver({t: 'pcm', pcm});
  const played = [];
  for (let q = 0; q < 15; q++) { // 1920 frames
    const out = outputs(1);
    p.process([], out);
    played.push(...out[0][0]);
  }
  for (let i = 0; i < 1920; i++)
    assert.strictEqual(played[i], i);
  const ticks = p.port.sent.filter(m => m.t === 'tick');
  assert.strictEqual(ticks.length, 2); // 1920 frames = 2 blocks of 960
  assert.strictEqual(ticks[1].read, 1920);
  assert.strictEqual(ticks[1].under, 0);
});

test('playback: an empty ring before any audio is not an underrun', () => {
  const p = new processors['cvc-playback']({processorOptions: {channels: 1}});
  for (let q = 0; q < 10; q++)
    p.process([], outputs(1));
  const ticks = p.port.sent.filter(m => m.t === 'tick');
  assert.ok(ticks.length >= 1);
  assert.strictEqual(ticks[ticks.length - 1].under, 0);
});

test('playback: running dry counts the silence and ramps back in', () => {
  const p = new processors['cvc-playback']({processorOptions: {channels: 1}});
  p.port.deliver({t: 'pcm', pcm: new Float32Array(200).fill(0.5)});
  const out1 = outputs(1);
  p.process([], out1); // 128 played
  const out2 = outputs(1);
  p.process([], out2); // 72 played, 56 silent
  assert.strictEqual(out2[0][0][71], 0.5);
  assert.strictEqual(out2[0][0][72], 0);
  for (let q = 0; q < 6; q++)
    p.process([], outputs(1)); // 768 silent: 960 rendered in all, a tick
  const tick = p.port.sent.filter(m => m.t === 'tick').pop();
  assert.strictEqual(tick.read, 200);
  assert.strictEqual(tick.under, 56 + 6 * 128);
  p.port.deliver({t: 'pcm', pcm: new Float32Array(256).fill(1)});
  const out3 = outputs(1);
  p.process([], out3);
  assert.ok(out3[0][0][0] < 0.1, 'the first sample after running dry is ramped');
  assert.strictEqual(out3[0][0][127], 1);
});

test('playback: stereo out of a mono ring', () => {
  const p = new processors['cvc-playback']({processorOptions: {channels: 1}});
  p.port.deliver({t: 'pcm', pcm: Float32Array.from({length: 128}, (_, i) => i)});
  const out = outputs(2);
  p.process([], out);
  assert.deepStrictEqual(Array.from(out[0][0]), Array.from(out[0][1]));
  assert.strictEqual(out[0][1][5], 5);
});

test('playback: a drop cross-fades across one quantum and skips the frames', () => {
  const p = new processors['cvc-playback']({processorOptions: {channels: 1}});
  // 0..4999 in the ring; drop 960
  p.port.deliver({t: 'pcm', pcm: Float32Array.from({length: 5000}, (_, i) => i)});
  p.port.deliver({t: 'drop', frames: 960});
  const out = outputs(1);
  p.process([], out);
  // Sample k fades from k to 960 + k.
  for (let k = 0; k < Q; k++) {
    const g = (k + 0.5) / Q;
    assert.ok(Math.abs(out[0][0][k] - ((1 - g) * k + g * (960 + k))) < 1e-3);
  }
  const next = outputs(1);
  p.process([], next);
  assert.strictEqual(next[0][0][0], 960 + Q); // it goes on from past the cut
  // read counts the skipped frames: after 960 rendered frames (here 256), a tick says so later.
  for (let q = 0; q < 6; q++)
    p.process([], outputs(1));
  const tick = p.port.sent.filter(m => m.t === 'tick').pop();
  assert.strictEqual(tick.read, 960 + 8 * Q);
});

test('playback: a drop waits until the ring holds it and a block after it', () => {
  const p = new processors['cvc-playback']({processorOptions: {channels: 1}});
  p.port.deliver({t: 'pcm', pcm: Float32Array.from({length: 1000}, (_, i) => i)});
  p.port.deliver({t: 'drop', frames: 960});
  const out = outputs(1);
  p.process([], out);
  assert.strictEqual(out[0][0][5], 5); // played on: not enough in the ring to cut
});

test('playback: more than the ring holds drops the oldest', () => {
  const p = new processors['cvc-playback']({processorOptions: {channels: 1, capacityFrames: 1000}});
  p.port.deliver({t: 'pcm', pcm: Float32Array.from({length: 1500}, (_, i) => i)});
  const out = outputs(1);
  p.process([], out);
  assert.strictEqual(out[0][0][0], 500);
});

test('playback: flush empties the ring; stop ends it', () => {
  const p = new processors['cvc-playback']({processorOptions: {channels: 1}});
  p.port.deliver({t: 'pcm', pcm: new Float32Array(1000).fill(1)});
  p.port.deliver({t: 'flush'});
  const out = outputs(1);
  p.process([], out);
  assert.strictEqual(out[0][0][0], 0);
  p.port.deliver({t: 'stop'});
  assert.strictEqual(p.process([], outputs(1)), false);
});

console.log(`cvc_audio_worklet_test: ${passed} passed`);
