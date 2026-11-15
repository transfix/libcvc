#!/usr/bin/env node
// tools/e2ee/txwtf_interop.js -- TX.WTF's own client (e2ee.js, messaging-e2ee.js) opening what
// cvc::net's E2eeStore sealed, and answering it. The middle step of tools/e2ee/interop.sh:
//
//   node tools/e2ee/txwtf_interop.js <txwtf>/txwtf/static/js <dir>
//
// <dir>/native.json (from e2ee_test's E2eeInterop case) holds jill's state in messaging-e2ee.js's
// key-store layout, made by the C++ store, and carl's three sealed messages as the server lists
// them. This loads jill into a TxwtfE2EEMemoryStore, opens each with decryptIncoming (the web
// page's code path), then replies twice with encryptAndSend through a stand-in API, and writes
// <dir>/js_reply.json: what it opened, and its replies as the server would list them for carl.
// Unlike txwtf_vectors.js, every random draw here is the system's.
'use strict';

const fs = require('fs');
const path = require('path');

function requireSodium() {
  for (const t of [path.join(__dirname, 'node_modules', 'libsodium-wrappers-sumo'),
                   'libsodium-wrappers-sumo']) {
    try {
      return require(t);
    } catch (e) {
      /* next */
    }
  }
  throw new Error('libsodium-wrappers-sumo not found: npm install libsodium-wrappers-sumo@0.7.15');
}

async function main() {
  const [jsDir, dir] = process.argv.slice(2);
  if (!jsDir || !dir) {
    console.error('usage: txwtf_interop.js <txwtf>/txwtf/static/js <dir>');
    process.exit(2);
  }
  const sodium = requireSodium();
  await sodium.ready;
  const lib = require(path.resolve(jsDir, 'e2ee.js'));
  const { TxwtfMessagingE2EE } = require(path.resolve(jsDir, 'messaging-e2ee.js'));
  const native = JSON.parse(fs.readFileSync(path.join(dir, 'native.json'), 'utf8'));

  const store = new lib.TxwtfE2EEMemoryStore();
  for (const [k, v] of Object.entries(native.jill_state))
    await store.set(k, v);
  const sent = [];
  const api = {
    publishE2EEKeys: async () => {
      throw new Error('jill has keys already');
    },
    getPreKeyBundle: async () => {
      throw new Error('jill has a session with carl already');
    },
    sendEncryptedMessage: async (convId, opts) => {
      const id = 900 + sent.length;
      sent.push({ id, convId, opts });
      return { id, conversation_id: convId, sender_id: 2 };
    },
  };
  const e2ee = new lib.TxwtfE2EE(sodium);
  const jill = new TxwtfMessagingE2EE({ e2ee, api, store });

  const opened = [];
  for (const m of native.messages) {
    const p = await jill.decryptIncoming(native.conv, m);
    opened.push(p.text);
    console.log(`e2ee.js opened #${m.id}: ${JSON.stringify(p.text)}`);
  }
  const replies = [];
  for (const text of ['hi carl, from e2ee.js', 'and once more, ✓']) {
    await jill.encryptAndSend(native.conv, { userId: 1, username: 'carl' }, text);
    const s = sent[sent.length - 1];
    const k = s.opts.recipient_keys.find((r) => r.recipient === 1);
    replies.push({
      text,
      message: {
        id: s.id, conversation_id: native.conv, sender_id: 2, algorithm: s.opts.algorithm,
        envelope: s.opts.envelope || null, is_sender: false, locked: false,
        ciphertext: s.opts.ciphertext, wrapped_key: k.wrapped_key, header: k.header,
      },
    });
    console.log(`e2ee.js sealed #${s.id}: ${JSON.stringify(text)}`);
  }
  fs.writeFileSync(path.join(dir, 'js_reply.json'), JSON.stringify({ opened, replies }, null, 1));
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
