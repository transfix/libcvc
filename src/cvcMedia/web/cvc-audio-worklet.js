// cvc-audio-worklet.js — cvcMedia's browser audio, on the page's audio rendering thread
// (cvc/media/web_audio.h; docs/roadmap/ARIADNE_CONFERENCE_ROADMAP.md §5j).
//
// Two AudioWorklet processors. They hold no wasm and share no memory with it, so they work in a
// single-threaded build and on a page that is not cross-origin isolated. Everything they move goes
// over the node's MessagePort, in 20 ms blocks:
//
//   cvc-capture   the mic (the node's input) out to the main thread: {t: 'cap', pcm}, interleaved
//                 f32, one block at a time, as the audio thread renders it.
//   cvc-playback  a ring of interleaved f32 the main thread fills ({t: 'pcm', pcm}), played out
//                 at the device's pace. Every block it reports {t: 'tick', read, under}: the frames
//                 it has taken from the ring (played, or skipped by a 'drop'), and the frames it
//                 played as silence because the ring was empty, both counted since it started.
//
// A message is a task on the main thread, and the browser runs it between the page's frames
// whether or not the tab is drawing them, and before timers it has throttled. So these ticks are
// the audio's clock there: a hidden tab, whose timers run about once a second and which draws no
// frames, still hears and sends 50 blocks a second.
//
// web_audio.cpp loads this file beside the page (cvc-audio-worklet.js) and, when the page has no
// such file, the copy compiled into the module.

const kBlockS = 0.02; // the block: 20 ms, as Opus packets are

// The mic, out in blocks.
class CvcCapture extends AudioWorkletProcessor {
  constructor(options) {
    super();
    const o = (options && options.processorOptions) || {};
    this.ch = o.channels > 0 ? o.channels : 1;
    this.block = o.blockFrames > 0 ? o.blockFrames : Math.round(sampleRate * kBlockS);
    this.buf = new Float32Array(this.block * this.ch);
    this.fill = 0; // frames in buf
    this.on = true;
    this.port.onmessage = e => {
      if (e.data && e.data.t === 'stop')
        this.on = false;
    };
  }

  process(inputs) {
    if (!this.on)
      return false;
    const input = inputs[0];
    // No channels: the mic is not connected yet, or its track ended. Nothing to send.
    const n = input && input.length > 0 ? input[0].length : 0;
    for (let i = 0; i < n;) {
      const take = Math.min(n - i, this.block - this.fill);
      for (let c = 0; c < this.ch; c++) {
        const src = input[Math.min(c, input.length - 1)];
        for (let k = 0; k < take; k++)
          this.buf[(this.fill + k) * this.ch + c] = src[i + k];
      }
      this.fill += take;
      i += take;
      if (this.fill === this.block) {
        this.port.postMessage({t: 'cap', pcm: this.buf}, [this.buf.buffer]);
        this.buf = new Float32Array(this.block * this.ch);
        this.fill = 0;
      }
    }
    return true;
  }
}

// The speaker: a ring the main thread keeps ahead of the device.
class CvcPlayback extends AudioWorkletProcessor {
  constructor(options) {
    super();
    const o = (options && options.processorOptions) || {};
    this.ch = o.channels > 0 ? o.channels : 1;
    this.block = o.blockFrames > 0 ? o.blockFrames : Math.round(sampleRate * kBlockS);
    this.cap = o.capacityFrames > 0 ? o.capacityFrames : Math.round(sampleRate * 2);
    this.ring = new Float32Array(this.cap * this.ch);
    this.r = 0;        // frames taken from the ring, ever (played, or skipped)
    this.w = 0;        // frames written to it, ever (w - r of them are in it)
    this.under = 0;    // frames played as silence while the ring was empty, since the first audio
    this.started = false; // the first audio has come (an empty ring before it is not an underrun)
    this.ramp = Math.max(1, Math.round(sampleRate * 0.0025)); // 2.5 ms back in after running dry
    this.fadeIn = 0;   // frames of that ramp left
    this.since = 0;    // frames rendered since the last tick
    this.cut = 0;      // frames to skip, cross-faded across one quantum, once the ring holds them
    this.on = true;
    this.port.onmessage = e => this.message(e.data || {});
  }

  message(m) {
    if (m.t === 'pcm' && m.pcm) {
      const frames = Math.floor(m.pcm.length / this.ch);
      // More than the ring holds: the oldest goes (only a push-mode caller far ahead does this).
      const over = this.w + frames - this.r - this.cap;
      if (over > 0)
        this.r += over;
      for (let f = 0; f < frames; f++) {
        const at = ((this.w + f) % this.cap) * this.ch;
        for (let c = 0; c < this.ch; c++)
          this.ring[at + c] = m.pcm[f * this.ch + c];
      }
      this.w += frames;
      this.started = true;
    } else if (m.t === 'drop' && m.frames > 0) {
      // Play less ahead: the next quantum cross-fades from where it is to FRAMES further on.
      this.cut = m.frames;
    } else if (m.t === 'flush') {
      this.r = this.w;
    } else if (m.t === 'stop') {
      this.on = false;
    }
  }

  process(inputs, outputs) {
    if (!this.on)
      return false;
    const out = outputs[0];
    const n = out.length > 0 ? out[0].length : 0;
    // A drop: this quantum fades out of the audio here and into the audio CUT frames on, and the
    // ring skips the CUT. Only with a block left after it; else it waits.
    if (this.cut > 0 && this.fadeIn === 0 && this.w - this.r >= this.cut + n + this.block) {
      for (let k = 0; k < n; k++) {
        const g = (k + 0.5) / n;
        const a = ((this.r + k) % this.cap) * this.ch, b = ((this.r + this.cut + k) % this.cap) * this.ch;
        for (let c = 0; c < out.length; c++) {
          const cc = Math.min(c, this.ch - 1);
          out[c][k] = (1 - g) * this.ring[a + cc] + g * this.ring[b + cc];
        }
      }
      this.r += this.cut + n;
      this.cut = 0;
      return this.tick(n);
    }
    const take = Math.min(n, this.w - this.r);
    for (let k = 0; k < take; k++) {
      const at = ((this.r + k) % this.cap) * this.ch;
      let g = 1;
      if (this.fadeIn > 0) {
        g = 1 - this.fadeIn / this.ramp;
        this.fadeIn--;
      }
      for (let c = 0; c < out.length; c++)
        out[c][k] = g * this.ring[at + Math.min(c, this.ch - 1)];
    }
    // The rest of the quantum is the zeros the output came with.
    if (take < n && this.started) {
      this.under += n - take;
      this.fadeIn = this.ramp;
    }
    this.r += take;
    return this.tick(n);
  }

  // N frames rendered: a tick each block.
  tick(n) {
    this.since += n;
    if (this.since >= this.block) {
      this.since -= this.block;
      this.port.postMessage({t: 'tick', read: this.r, under: this.under});
    }
    return true;
  }
}

registerProcessor('cvc-capture', CvcCapture);
registerProcessor('cvc-playback', CvcPlayback);
