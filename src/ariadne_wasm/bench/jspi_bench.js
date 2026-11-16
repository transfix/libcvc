// jspi_bench.js: the page of the JSPI-against-Asyncify bench (jspi_bench.mjs serves it and reads
// window.benchResult). It runs jspi_guest.c's workloads four ways:
//
//   plain     the guest as clang built it, called straight through (the reference)
//   metered   metered (ari_wasm_meter) and -O1, called straight through: what a JSPI guest runs
//   asyncify  metered, then Asyncify, as cvc_ariadne_wasm_transform installs a guest today
//   jspi      the metered guest, its ari.gas a WebAssembly.Suspending and its export promising
//
// unsliced (gas enough for the whole call), then sliced: each slice of gas ends at ari.gas, where
// the Asyncify guest unwinds into its room and is rewound by the next call, as
// browser_runtime.cpp does it, and the JSPI guest suspends and is resumed through the event loop.
//
// Three sections:
//   workloads  each workload unsliced four ways, then sliced at SLICES (64,000 gas is the default
//              slice: wasm_slice_steps 64 at a thousand gas a step)
//   pauses     spin and deep (the same loop 48 frames down) in slices so small that the pauses
//              are most of the time: a pause's own cost, at either depth
//   hosts      jspi_host.cpp, under -sASYNCIFY (as ariglx is) or -sJSPI, with no exceptions,
//              Emscripten's JS ones (as libcvc builds) or wasm's (jspi_bench.mjs's HOSTS), polls
//              the guest's spin from `depth` frames down (34 is ariglx's at a guest's slice): at
//              once for an Asyncify guest, waiting for a JSPI one
//
// only= picks sections, comma-separated (all three by default).
//
// Each case is a set of variants timed in rounds, each round timing every variant once, so a
// change in the machine's load falls on all of them alike; a case's time is its median round.

const q = new URLSearchParams(location.search);
const num = (k, d) => (q.has(k) ? Number(q.get(k)) : d);
const REPS = num('reps', 9);
const WORK = {
  spin: num('spin', 20000000),
  fib: num('fib', 30),
  sort: num('sort', 300000),
  sha: num('sha', 4096),
  interp: num('interp', 2000000),
};
const SLICES = (q.get('slices') || '64000,8000').split(',').map(Number);
const PAUSE_ARG = num('pausearg', 2000000);
const PAUSE_SLICES = (q.get('pauseslices') || '1000').split(',').map(Number);
const DEPTHS = (q.get('depths') || '1,34,128').split(',').map(Number);
const HOST_SLICES = (q.get('hostslices') || '64000,1000').split(',').map(Number);
const HOSTS = (q.get('hosts') || 'asyncify,asyncify-eh,jspi,jspi-eh,jspi-wasm-eh').split(',');
const BIG = 1n << 62n;
const EXPORTS = [...Object.keys(WORK), 'deep'];
const ONLY = (q.get('only') || 'workloads,pauses,hosts').split(',');

const log = [];
const say = s => { log.push(s); document.getElementById('log').textContent = log.join('\n'); };

const median = a => { const s = [...a].sort((x, y) => x - y); return s[s.length >> 1]; };
// The variants of a case, {name: fn}, warmed once, then timed in REPS rounds: {name: {ms, out}}.
async function rounds(variants) {
  const names = Object.keys(variants), t = {}, out = {};
  for (const n of names) {
    out[n] = await variants[n](); // warm: tiered up, caches filled
    t[n] = [];
  }
  for (let i = 0; i < REPS; ++i)
    for (const n of names) {
      const t0 = performance.now();
      out[n] = await variants[n]();
      t[n].push(performance.now() - t0);
    }
  const r = {};
  for (const n of names) r[n] = {ms: median(t[n]), out: out[n]};
  return r;
}
async function bytesOf(name) {
  const r = await fetch(name, {cache: 'no-store'});
  if (!r.ok) throw new Error(`${name}: ${r.status}`);
  return new Uint8Array(await r.arrayBuffer());
}
async function gzipSize(b) {
  const s = new Blob([b]).stream().pipeThrough(new CompressionStream('gzip'));
  return (await new Response(s).arrayBuffer()).byteLength;
}

// ── the guests ──

function plainGuest(mod) {
  const ex = new WebAssembly.Instance(mod, {}).exports;
  ex._initialize();
  return {call: (name, arg) => ex[name](arg) >>> 0};
}

// The metered guest called straight through, with gas for the whole call.
function meteredGuest(mod) {
  const ex = new WebAssembly.Instance(mod, {ari: {gas() { throw new Error('out of gas'); }}}).exports;
  ex._initialize();
  return {
    call(name, arg) {
      ex.ari_gas.value = BIG;
      const r = ex[name](arg) >>> 0;
      return {r, gas: BIG - ex.ari_gas.value};
    },
  };
}

// Asyncify, as browser_runtime.cpp's cvc_wasm_call drives it.
function asyncifyGuest(mod) {
  const st = {rewinding: false, pending: 0n, buf: 0, size: 0};
  let ex;
  const gas = cost => {
    if (st.rewinding) {
      st.rewinding = false;
      ex.asyncify_stop_rewind();
      return;
    }
    st.pending = cost;
    const m = new Uint32Array(ex.memory.buffer, st.buf, 2);
    m[0] = st.buf + 8;
    m[1] = st.buf + st.size;
    ex.asyncify_start_unwind(st.buf);
  };
  ex = new WebAssembly.Instance(mod, {ari: {gas}}).exports;
  ex._initialize();
  st.buf = ex.room();
  st.size = ex.room_size();
  // One call of `name`, a slice at a time; poll() runs the next slice and says whether it paused.
  const start = (name, arg, slice) => {
    const f = ex[name], s = BigInt(slice);
    const c = {paused: false, slices: 0, gas: 0n, r: 0};
    c.poll = () => {
      const set = c.paused && st.pending > s ? st.pending : s;
      ex.ari_gas.value = set;
      if (c.paused) {
        st.rewinding = true;
        ex.asyncify_start_rewind(st.buf);
      }
      const r = f(arg);
      c.gas += set - ex.ari_gas.value;
      if (ex.asyncify_get_state() == 1) {
        ex.asyncify_stop_unwind();
        c.paused = true;
        ++c.slices;
        return 1;
      }
      if (st.rewinding) throw new Error('the guest did not go back to where it paused');
      c.r = r >>> 0;
      return 0;
    };
    return c;
  };
  return {
    start,
    unsliced(name, arg) {
      ex.ari_gas.value = BIG;
      return ex[name](arg) >>> 0;
    },
    run(name, arg, slice) {
      const c = start(name, arg, slice);
      while (c.poll()) {}
      return c;
    },
  };
}

// JSPI: ari.gas suspends the guest on a promise that the next poll resolves, and the guest goes on
// in a microtask. poll() resolves when the slice it started has ended, at ari.gas or the return.
function jspiGuest(mod) {
  const st = {pending: 0n, resume: null, paused: null};
  const gas = new WebAssembly.Suspending(cost => {
    st.pending = cost;
    return new Promise(res => {
      st.resume = res;
      st.paused();
    });
  });
  const ex = new WebAssembly.Instance(mod, {ari: {gas}}).exports;
  ex._initialize();
  const promising = {};
  for (const name of EXPORTS) promising[name] = WebAssembly.promising(ex[name]);
  const start = (name, arg, slice) => {
    const s = BigInt(slice);
    const c = {slices: 0, gas: 0n, r: 0, done: false, error: null, set: 0n, started: false};
    let wake = null;
    st.paused = () => wake(1);
    c.poll = () => new Promise(res => {
      wake = v => {
        c.gas += c.set - ex.ari_gas.value;
        if (v) ++c.slices;
        res(v);
      };
      c.set = c.started && st.pending > s ? st.pending : s;
      ex.ari_gas.value = c.set;
      if (!c.started) {
        c.started = true;
        promising[name](arg).then(r => { c.r = r >>> 0; c.done = true; wake(0); },
                                  e => { c.error = e; c.done = true; wake(0); });
      } else {
        st.resume();
      }
    });
    return c;
  };
  return {
    start,
    async unsliced(name, arg) {
      ex.ari_gas.value = BIG;
      return (await promising[name](arg)) >>> 0;
    },
    async run(name, arg, slice) {
      const c = start(name, arg, slice);
      while (await c.poll()) {}
      if (c.error) throw c.error;
      return c;
    },
  };
}

// ── the hosts ──

async function loadHost(script, factory) {
  await new Promise((res, rej) => {
    const s = document.createElement('script');
    s.src = script;
    s.onload = res;
    s.onerror = () => rej(new Error(script + ' did not load'));
    document.head.appendChild(s);
  });
  return window[factory]({});
}


async function main() {
  const env = {
    userAgent: navigator.userAgent,
    jspi: typeof WebAssembly.Suspending == 'function' && typeof WebAssembly.promising == 'function',
    crossOriginIsolated: self.crossOriginIsolated,
    reps: REPS, work: WORK, slices: SLICES, pauseArg: PAUSE_ARG, pauseSlices: PAUSE_SLICES,
    depths: DEPTHS, hostSlices: HOST_SLICES,
  };
  window.benchResult = {env, sizes: {}, compile: {}, workloads: [], pauses: [], hosts: []};
  const R = window.benchResult;
  if (!env.jspi) throw new Error('this browser has no JSPI (WebAssembly.Suspending, promising)');

  const files = {plain: 'guest.plain.wasm', metered: 'guest.jspi.wasm',
                 asyncify: 'guest.asyncify.wasm'};
  const bytes = {}, mods = {}, compiles = {};
  for (const [k, f] of Object.entries(files)) {
    bytes[k] = await bytesOf(f);
    R.sizes[k] = {raw: bytes[k].byteLength, gzip: await gzipSize(bytes[k])};
    compiles[k] = () => WebAssembly.compile(bytes[k]);
  }
  const c = await rounds(compiles);
  for (const k of Object.keys(files)) {
    R.compile[k] = c[k].ms;
    mods[k] = c[k].out;
  }
  say('sizes ' + JSON.stringify(R.sizes) + ' compile ' + JSON.stringify(R.compile));

  const plain = plainGuest(mods.plain), metered = meteredGuest(mods.metered);
  const asy = asyncifyGuest(mods.asyncify), jspi = jspiGuest(mods.metered);
  const expect = (what, r, want) => {
    if (r !== want) throw new Error(`${what}: ${r}, not ${want}`);
  };
  // A sliced case with its unsliced baselines in the same rounds: the times, and the pauses.
  const sliced = async (name, arg, slice, want, gas) => {
    const r = await rounds({
      asyncify0: () => asy.unsliced(name, arg),
      asyncify: () => asy.run(name, arg, slice),
      jspi0: () => jspi.unsliced(name, arg),
      jspi: () => jspi.run(name, arg, slice),
    });
    expect(`${name} asyncify at ${slice}`, r.asyncify.out.r, want);
    expect(`${name} jspi at ${slice}`, r.jspi.out.r, want);
    if (gas !== undefined && (Number(r.asyncify.out.gas) !== gas || Number(r.jspi.out.gas) !== gas))
      throw new Error(`${name} at ${slice}: gas ${r.asyncify.out.gas} and ${r.jspi.out.gas}, not ${gas}`);
    return {slice, pauses: r.asyncify.out.slices, jspiPauses: r.jspi.out.slices,
            asyncify0: r.asyncify0.ms, asyncify: r.asyncify.ms, jspi0: r.jspi0.ms, jspi: r.jspi.ms};
  };

  for (const [name, arg] of ONLY.includes('workloads') ? Object.entries(WORK) : []) {
    const u = await rounds({
      plain: () => plain.call(name, arg),
      metered: () => metered.call(name, arg),
      asyncify: () => asy.unsliced(name, arg),
      jspi: () => jspi.unsliced(name, arg),
    });
    const w = {name, arg, result: u.plain.out, gas: Number(u.metered.out.gas), unsliced: {}, sliced: []};
    expect(`${name} metered`, u.metered.out.r, w.result);
    expect(`${name} asyncify`, u.asyncify.out, w.result);
    expect(`${name} jspi`, u.jspi.out, w.result);
    for (const k of Object.keys(u)) w.unsliced[k] = u[k].ms;
    for (const slice of SLICES) w.sliced.push(await sliced(name, arg, slice, w.result, w.gas));
    R.workloads.push(w);
    say(`${name} ${JSON.stringify(w)}`);
  }

  // A pause's own cost: slices so small that the pauses are most of the time, shallow and deep.
  for (const name of ONLY.includes('pauses') ? ['spin', 'deep'] : []) {
    const want = plain.call(name, PAUSE_ARG);
    for (const slice of PAUSE_SLICES) {
      const p = {name, arg: PAUSE_ARG, ...await sliced(name, PAUSE_ARG, slice, want)};
      R.pauses.push(p);
      say(`pause ${JSON.stringify(p)}`);
    }
  }

  // The hosts: each polls the guest's spin, a slice at a time, from `depth` frames down.
  if (!ONLY.includes('hosts')) return;
  const want = plain.call('spin', PAUSE_ARG);
  const polled = async (host, guest, depth, wait, slice) => {
    const c = guest.start('spin', PAUSE_ARG, slice);
    host.benchPoll = () => c.poll();
    await host.ccall('run', 'number', ['number', 'number'], [depth, wait], {async: true});
    if (c.error) throw c.error;
    return c;
  };
  for (const hostName of HOSTS) {
    const factory = 'host_' + hostName.replace(/-/g, '_');
    let host = await loadHost(factory + '.js', factory);
    // Whether the host runs at all from two frames down, and whether it can wait for a JSPI
    // guest there. JSPI can't suspend a stack with JS frames on it, and under JSPI Emscripten makes
    // every invoke_* (its JS exceptions' calls) a suspending import, so V8 refuses one called from
    // below another, waiting or not. A host that failed is left broken, so each try has an instance
    // of its own. One that can't wait is timed with the Asyncify guest alone; one that can't run
    // is only recorded.
    const tries = {};
    for (const [what, guest, wait] of [['run', asy, 0], ['wait', jspi, 1]]) {
      try {
        await polled(host, guest, 2, wait, 64000);
      } catch (e) {
        tries[what] = String((e && e.message) || e);
      }
      host = await window[factory]({});
    }
    if (tries.run) {
      R.hosts.push({host: hostName, error: tries.run, runs: false});
      say(`host ${hostName} doesn't run: ${tries.run}`);
      continue;
    }
    const error = tries.wait || null;
    for (const slice of HOST_SLICES)
      for (const depth of DEPTHS) {
        const variants = {
          asyncify0: () => asy.unsliced('spin', PAUSE_ARG),
          asyncifyGuest: () => polled(host, asy, depth, 0, slice),
        };
        if (!error) {
          variants.jspi0 = () => jspi.unsliced('spin', PAUSE_ARG);
          variants.jspiGuest = () => polled(host, jspi, depth, 1, slice);
        }
        const r = await rounds(variants);
        expect(`${hostName} host, asyncify guest`, r.asyncifyGuest.out.r, want);
        if (!error) expect(`${hostName} host, jspi guest`, r.jspiGuest.out.r, want);
        const row = {host: hostName, slice, depth, pauses: r.asyncifyGuest.out.slices,
                     asyncify0: r.asyncify0.ms, asyncifyGuest: r.asyncifyGuest.ms,
                     jspi0: error ? null : r.jspi0.ms, jspiGuest: error ? null : r.jspiGuest.ms,
                     error};
        R.hosts.push(row);
        say(`host ${JSON.stringify(row)}`);
      }
  }
}

main().then(() => { window.benchDone = 'ok'; say('done'); },
            e => { window.benchDone = 'error: ' + (e && e.stack || e); say(window.benchDone); });
