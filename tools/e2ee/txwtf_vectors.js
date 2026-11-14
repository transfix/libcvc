#!/usr/bin/env node
// tools/e2ee/txwtf_vectors.js -- write the interop vectors for cvc::net's E2EE port
// (src/cvc/tests/data/e2ee/txwtf_vectors.json) from TX.WTF's own client crypto.
//
//   node tools/e2ee/txwtf_vectors.js <txwtf>/txwtf/static/js <out.json>
//
// It loads txwtf's e2ee.js and messaging-e2ee.js unmodified, with libsodium-wrappers-sumo (the
// build the TX.WTF page loads, 0.7.15; install it with `npm install libsodium-wrappers-sumo@0.7.15`
// next to this script or anywhere on NODE_PATH), and runs three users through an in-memory stand-in
// for the server, as txwtf's own frontend tests do.
//
// Every random draw comes from a seeded stream instead of the system's, so a port that draws in
// the same order reproduces every byte: libcvc's tests replay the transcript with cvc::net's
// E2eeStore and compare each published bundle, sealed message and backup blob, and open every
// message on the other side. The stream, per user: 64-byte blocks BLAKE2b-512(key = seed,
// message = the block's index as 8 little-endian bytes), taken in order. Through it:
//   randombytes_buf(n)        the next n bytes;
//   randombytes_uniform(n)    the next 4 bytes as a little-endian uint32, mod n;
//   crypto_box_keypair()      crypto_box_seed_keypair(the next 32 bytes);
//   crypto_sign_keypair()     crypto_sign_seed_keypair(the next 32 bytes).
// Nothing else e2ee.js calls draws randomness.
'use strict';

const fs = require('fs');
const path = require('path');
const crypto = require('crypto');

function requireSodium() {
  const tries = [path.join(__dirname, 'node_modules', 'libsodium-wrappers-sumo'),
                 'libsodium-wrappers-sumo'];
  for (const t of tries) {
    try {
      return require(t);
    } catch (e) {
      /* next */
    }
  }
  throw new Error('libsodium-wrappers-sumo not found: npm install libsodium-wrappers-sumo@0.7.15');
}

function seededSodium(sodium, seed) {
  let counter = 0;
  let pool = new Uint8Array(0);
  const take = (n) => {
    while (pool.length < n) {
      const idx = new Uint8Array(8);
      let x = counter++;
      for (let i = 0; i < 8; i++) {
        idx[i] = x % 256;
        x = Math.floor(x / 256);
      }
      const block = sodium.crypto_generichash(64, idx, seed);
      const next = new Uint8Array(pool.length + block.length);
      next.set(pool);
      next.set(block, pool.length);
      pool = next;
    }
    const out = pool.slice(0, n);
    pool = pool.slice(n);
    return out;
  };
  const s = Object.create(sodium);
  s.randombytes_buf = (n) => take(n);
  s.randombytes_uniform = (n) => {
    const b = take(4);
    return (b[0] + b[1] * 256 + b[2] * 65536 + b[3] * 16777216) % n;
  };
  s.crypto_box_keypair = () => sodium.crypto_box_seed_keypair(take(32));
  s.crypto_sign_keypair = () => sodium.crypto_sign_seed_keypair(take(32));
  return s;
}

// The E2EE server endpoints, in memory (txwtf's frontend/__tests__/messaging-e2ee.test.js keeps
// the same shapes, and txwtf/core/e2ee_dm.py's _serialize_message the message dict).
class FakeServer {
  constructor() {
    this.keys = {};
    this.convs = {};
    this.msgId = 100;
  }
  publish(username, userId, p) {
    this.keys[username] = {
      userId,
      registration_id: p.registration_id,
      identity_key_ed25519: p.identity_key_ed25519,
      identity_key_x25519: p.identity_key_x25519,
      signed_prekey: p.signed_prekey,
      opks: (p.one_time_prekeys || []).map((k) => ({ ...k, consumed: false })),
    };
  }
  bundle(username) {
    const k = this.keys[username];
    let opk = null;
    const a = k.opks.find((o) => !o.consumed);
    if (a) {
      a.consumed = true;
      opk = { key_id: a.key_id, public_key: a.public_key };
    }
    return {
      user_id: k.userId,
      username,
      registration_id: k.registration_id,
      identity_key_ed25519: k.identity_key_ed25519,
      identity_key_x25519: k.identity_key_x25519,
      signed_prekey: k.signed_prekey,
      one_time_prekey: opk,
    };
  }
  send(convId, senderId, opts) {
    const id = ++this.msgId;
    (this.convs[convId] = this.convs[convId] || []).push({ id, convId, senderId, opts });
    return { id, conversation_id: convId, sender_id: senderId };
  }
  // The message as GET /e2ee/dm/conversations/{id}/messages gives it to `viewerId`.
  view(id, viewerId) {
    for (const list of Object.values(this.convs)) {
      for (const m of list) {
        if (m.id !== id)
          continue;
        const isSender = m.senderId === viewerId;
        const kr = m.opts.recipient_keys.find((r) => r.recipient === viewerId);
        return {
          id: m.id,
          conversation_id: m.convId,
          sender_id: m.senderId,
          algorithm: m.opts.algorithm,
          envelope: m.opts.envelope || null,
          is_paid_content: false,
          is_sender: isSender,
          locked: false,
          ciphertext: isSender || kr ? m.opts.ciphertext : null,
          wrapped_key: !isSender && kr ? kr.wrapped_key : null,
          header: !isSender && kr ? kr.header : null,
        };
      }
    }
    throw new Error('no message ' + id);
  }
}

async function main() {
  const [jsDir, out] = process.argv.slice(2);
  if (!jsDir || !out) {
    console.error('usage: txwtf_vectors.js <txwtf>/txwtf/static/js <out.json>');
    process.exit(2);
  }
  const sodium = requireSodium();
  await sodium.ready;
  const e2eePath = path.resolve(jsDir, 'e2ee.js');
  const msgPath = path.resolve(jsDir, 'messaging-e2ee.js');
  const lib = require(e2eePath);
  const { TxwtfMessagingE2EE } = require(msgPath);
  const sha = (p) => crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
  const b64 = (u8) => sodium.to_base64(u8, sodium.base64_variants.ORIGINAL);
  const seed = (name) => sodium.crypto_generichash(32, sodium.from_string('cvc-e2ee-vectors:' + name));

  const server = new FakeServer();
  const steps = [];
  const users = {};
  const mkUser = async (name, userId, opkCount) => {
    const e2ee = new lib.TxwtfE2EE(seededSodium(sodium, seed(name)));
    const store = new lib.TxwtfE2EEMemoryStore();
    const api = {
      publishE2EEKeys: async (p) => {
        server.publish(name, userId, p);
        steps.push({ op: 'publish', who: name, opk_count: opkCount, payload: p });
      },
      getPreKeyBundle: async (username) => {
        const b = server.bundle(username);
        steps.push({ op: 'bundle', who: name, peer: username, bundle: b });
        return b;
      },
      sendEncryptedMessage: async (convId, opts) => {
        const r = server.send(convId, userId, opts);
        steps.push({ op: 'send', who: name, conv: convId, id: r.id, text: pending.text,
                     group: !!pending.group, body: opts });
        return r;
      },
    };
    let pending = {};
    const m = new TxwtfMessagingE2EE({ e2ee, api, store });
    await m.ensureIdentity(opkCount);
    users[name] = { name, userId, m, e2ee, store, setPending: (p) => (pending = p) };
  };
  await mkUser('alice', 1, 2);
  await mkUser('bob', 2, 1);
  await mkUser('carol', 3, 2);

  const send = async (from, to, conv, text) => {
    const u = users[from];
    u.setPending({ text });
    const r = await u.m.encryptAndSend(conv, { userId: users[to].userId, username: to }, text);
    return r.id;
  };
  const sendGroup = async (from, members, conv, text) => {
    const u = users[from];
    u.setPending({ text, group: true });
    const r = await u.m.encryptAndSendGroup(
        conv, members.map((n) => ({ userId: users[n].userId, username: n })), text);
    return r.id;
  };
  const recv = async (who, conv, id) => {
    const message = server.view(id, users[who].userId);
    const payload = await users[who].m.decryptIncoming(conv, message);
    steps.push({ op: 'open', who, conv, id, message, text: payload.text,
                 attachments: payload.attachments });
  };

  // conv 10: alice starts with bob (bob's one OPK), three messages, bob opens them 3, 1, 2.
  const m1 = await send('alice', 'bob', 10, 'hello bob');
  const m2 = await send('alice', 'bob', 10, 'second');
  const m3 = await send('alice', 'bob', 10,
                        'third ✓ ünïcödé \u{1F512} "quotes" \\ / \n tab\t ctrl\u0001 del\u007f');
  await recv('bob', 10, m3);
  await recv('bob', 10, m1);
  await recv('bob', 10, m2);
  // A ratchet turn each way; alice's next send carries no envelope.
  const m4 = await send('bob', 'alice', 10, 'hi alice');
  await recv('alice', 10, m4);
  const m5 = await send('alice', 'bob', 10, 'after your reply');
  const m6 = await send('alice', 'bob', 10, 'and one more');
  await recv('bob', 10, m5);
  await recv('bob', 10, m6);
  const m7 = await send('bob', 'alice', 10, '');
  await recv('alice', 10, m7);
  // conv 12: carol starts with bob, whose OPK pool is empty now (opk_id null).
  const m8 = await send('carol', 'bob', 12, 'from carol, no one-time key');
  await recv('bob', 12, m8);
  // conv 20: a group (grp-dr-v1) from alice to bob and carol, then bob to alice and carol.
  const g1 = await sendGroup('alice', ['bob', 'carol'], 20, 'group hello');
  await recv('bob', 20, g1);
  await recv('carol', 20, g1);
  const g2 = await sendGroup('bob', ['alice', 'carol'], 20, 'group reply');
  await recv('alice', 20, g2);
  await recv('carol', 20, g2);

  // Tampering, checked on the C++ side only (e2ee.js would corrupt its session: see the README).
  const genuine = server.view(m6, 2);
  const flip = (s, i) => {
    const raw = sodium.from_base64(s, sodium.base64_variants.ORIGINAL);
    raw[i] ^= 0x01;
    return b64(raw);
  };

  // The passphrase backup of bob's identity and signed prekey (Argon2id at e2ee.js's MODERATE).
  const bobMem = await users.bob.store.get('identity');
  const bobKeys = users.bob.m._deserializeIdentity(bobMem);
  const exported = users.bob.e2ee.exportSecretBundle(bobKeys.identity, bobKeys.spk);
  const backupE2ee = new lib.TxwtfE2EE(seededSodium(sodium, seed('backup')));
  const passphrase = 'correct horse battery staple é';
  const blob = backupE2ee.wrapPrivateBundle(passphrase, exported);
  const back = users.bob.e2ee.unwrapPrivateBundle(passphrase, blob);
  if (JSON.stringify(back) !== JSON.stringify(exported))
    throw new Error('backup round trip');

  const stores = {};
  for (const u of Object.values(users)) {
    const kv = {};
    for (const key of await u.store.keys())
      kv[key] = await u.store.get(key);
    stores[u.name] = kv;
  }

  const doc = {
    about: 'TX.WTF E2EE interop vectors for cvc::net (tools/e2ee/txwtf_vectors.js)',
    source: {
      e2ee_js_sha256: sha(e2eePath),
      messaging_e2ee_js_sha256: sha(msgPath),
      libsodium_js: 'libsodium-wrappers-sumo ' +
                    require(require.resolve('libsodium-wrappers-sumo/package.json')).version +
                    ' (libsodium ' + sodium.SODIUM_VERSION_STRING + ')',
    },
    seed_rule: 'seed(name) = BLAKE2b-256("cvc-e2ee-vectors:" + name); the stream as this script says',
    seeds: Object.fromEntries(['alice', 'bob', 'carol', 'backup'].map((n) => [n, b64(seed(n))])),
    users: Object.fromEntries(Object.values(users).map((u) => [u.name, u.userId])),
    steps,
    tamper: {
      who: 'bob',
      conv: 10,
      genuine_id: m6,
      wrapped_key_flipped: { ...genuine, wrapped_key: flip(genuine.wrapped_key, 30) },
      body_flipped: { ...genuine, ciphertext: flip(genuine.ciphertext, 40) },
    },
    backup: { passphrase, bundle: exported, blob },
    final_stores: stores,
  };
  fs.writeFileSync(out, JSON.stringify(doc, null, 1) + '\n');
  console.log(`wrote ${out}: ${steps.length} steps (${steps.filter((s) => s.op === 'send').length} sends)`);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
