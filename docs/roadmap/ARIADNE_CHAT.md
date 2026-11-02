# Ariadne conference chat: room chat and direct messages

*Design note for room chat, direct messages (DMs), and the two widgets they need. It belongs with
[ARIADNE_CONFERENCE_ROADMAP.md](ARIADNE_CONFERENCE_ROADMAP.md), which covers the room tree, the
relay, bound writes (§5d) and the area of interest (S7).*

## Goals

- **Room chat.** Everyone in the room sees it. A late joiner can scroll up through earlier
  messages. The client fetches older messages a page at a time as the view scrolls up, so the
  room's whole history is never replicated to every client up front.
- **Direct messages** between two participants, routed through the relay. No other client receives
  them, whatever it connects as.
- **Two widgets:** `text_view:`, a scrollable multi-line text buffer that asks for more when
  scrolled to the top, and `message_input:`, a post field that is single-line by default and takes
  Shift+Enter for a new line. Both work in the ImGui backend and in the terminal backends (FTXUI,
  and tvision, which hosts FTXUI's widgets).
- **Reuse.** The chat window and the users list are components (`cvc://components/...`). Their
  windows are `units:` and their logic is `functions:` in the same library, so another app imports
  both.

## Why the messages are not plain room-tree writes

Everything in the relay's `conf` shard goes to every client. A joiner's admit snapshot holds the
whole tree, the WebSocket re-broadcast sends it again, and a write a client makes in its own
participant subtree is forwarded to every other connection. The S7 delivery filters narrow that per
receiver, but only for a connection that has joined and been given a filter. A connection that
never joins, or that the admission hold times out, gets everything, and with `--aoi off` there are
no filters at all. History kept in the shared tree would therefore reach every client in full. A
DM kept there, or posted through the poster's own subtree, would reach every client too.

So the relay keeps the history, and a client's request never enters the shared tree:

- **A client to the relay:** a `state_message` with `ttl_hops = 0` on the client's own path
  `conf.rooms.<room>.participants.<pid>.chat`. Bound writes already limit a client's messages to its
  own participant path. The relay now honours `ttl_hops == 0` (as `state_message.h` defined it, "do
  not forward across peers"): it takes the message itself and neither forwards nor bridges it. The
  room's write gate refuses a chat message with any other `ttl_hops`, so a client cannot have its
  request fanned out.
- **The relay to one client:** a targeted push (`push_snapshot_to_holder`, a variant of
  `push_snapshot_to`). It writes only to the connection that holds the pid under bound writes, so
  a second connection that claims a live participant's pid gets nothing. Pushed entries land in
  the receiver's tree as an unsolicited snapshot does. They are not journaled there, so they are
  never sent back.

## State layout

What each client holds under `conf.rooms.<room>`:

| path | written by | reaches |
|---|---|---|
| `chat.epoch` | relay, once at start | everyone (replicated) |
| `chat.room.head`, `chat.room.first` | relay | everyone (replicated): the next seq, the oldest one kept |
| `chat.room.m.<seq>` | relay (push) | present participants as it is posted; a page to the client that asked |
| `chat.dms` | relay (push) | that participant only: the peers it has a conversation with |
| `chat.dm.<peer>.head`, `.first`, `.m.<seq>` | relay (push) | only the two participants of the conversation, each under the other's pid |
| `chat.error` | relay (push) | the poster only: why its last post was refused |
| `chat.rev` | relay (push, last entry of every push) | that participant only: changes on each push |

A message leaf is one JSON value, so it arrives in one piece:
`{"seq":41,"from":"<pid>","name":"alice","t":1730512345.2,"text":"hi","e":"<epoch>"}`. `seq`
counts up from 0 per channel (the room, or one conversation). `t` is the relay's clock. `e`
(the epoch) lets a client discard what an earlier relay process pushed.

A client reads two leaves per tick, `chat.rev` and `chat.room.head`, and looks further into the
tree only when one of them has moved. This follows the revision-leaf rule from the avatar
catalog: a per-frame reader checks one leaf, never a subtree.

The client's requests (the JSON in the message's `string_value`):

- `{"op":"post","cseq":7,"to":"","text":"hello"}`. `to` is `""` for the room, or a pid for a DM.
  `cseq` is the client's own count, which comes back in `chat.error`.
- `{"op":"page","channel":"","before":120}`. Push the page of messages before seq 120 (`channel`
  is `""` or the peer's pid).

## Pagination

- **Page size:** 32 messages. The relay keeps the last 5000 room messages and the last 1000 of
  each conversation, in memory.
- **First view:** a client looking at a channel with `head > 0` and nothing loaded asks for the page
  before `head`. A late joiner sees the last 32 messages, and `first` tells it how far back it can
  go.
- **Scrolling back:** `text_view`'s `on_top:` runs when the view reaches the top. The chat
  component answers with `(chat-more ...)`, which asks for the page before the oldest message held.
  Only one request per channel is in flight at a time. One unanswered after 3 s is sent again.
- **Live messages** are pushed to every present participant as they are posted.
- **Gaps:** a client shows only the contiguous run that ends at the newest message it holds. When a
  push is lost (a stalled WebSocket skips pushes past 2 MB of backlog) and `head` runs ahead of what
  it holds for 2 s, it asks for the newest page again. The relay re-pushes each participant's
  conversation heads every 5 s, so a lost DM is noticed too.

## DM routing and permissions

- **Who may post:** a participant, as itself. The message's origin must be the pid its connection
  holds (bound writes), and the relay stamps the sender's pid and current name, so a message cannot
  claim another sender.
- **Who may read a DM:** the relay pushes it only to the connections holding the two pids. Each one
  receives it under the other's pid (`chat.dm.<peer>`), the sender as its own confirmation.
  Observers, connections that have not joined, and a second connection claiming either pid get
  nothing. Nothing about a conversation is in the shared tree.
- **What the relay refuses** (reported in the sender's `chat.error`): text that is empty or longer
  than 2000 bytes; a DM to someone not in the room, or to oneself; more than 64 conversations per
  participant; and posting faster than the rate limit (a burst of 10, refilled at 2 a second).
  Control characters other than newline and tab are dropped from the text.
- **`--bind-writes off`:** without bound writes, a connection can speak as any pid and read that
  pid's DMs, so arisrv turns DMs off. Room chat still works.
- **Limits of the design:** a DM is private from the other clients, not from the relay that routes
  it (it is not end-to-end encrypted). DMs are addressed by pid, which is per session, so a
  conversation ends with the session; the relay drops a conversation once neither participant is
  present. History is in the relay's memory and is lost when it restarts (the epoch tells clients).

## The DSL surface

Room verbs, registered with `register_room_intrinsics`. `ROOM` is the `room-join` handle.
`CHANNEL` is `""` for the room, or a peer's pid.

- `(chat-post ROOM TEXT [TO])` returns `{ok}`, or `{ok #f error}` when the text is empty or too long.
- `(chat-info ROOM [CHANNEL])` returns `{rev count head first more loading error}`. It also makes the
  first-view and gap requests. `rev` changes whenever the messages held for the channel change.
- `(chat-messages ROOM [CHANNEL])` returns the held messages, oldest first, as dicts:
  `{seq from name text t clock mine}`.
- `(chat-more ROOM [CHANNEL])` asks for the previous page and returns `{ok more}`.
- `(chat-channels ROOM)` returns the conversations: `{peer name head}`.

The widgets:

```yaml
- text_view: Chat
  bind: chat.view.text          # the text (lines split on \n)
  height: 240                   # px (rows in a terminal); 0: the space left in the window
  follow: true                  # stay at the bottom while new lines arrive, if already there
  on_top: (chat-ui-more "chat") # runs when the view is scrolled to the top
- message_input: Message
  bind: chat.draft              # the draft; cleared when it is sent
  hint: Say something           # shown while it is empty
  multiline: true               # Shift+Enter (Alt+Enter or Ctrl+J in a terminal) adds a line
  on: (chat-ui-send "chat" (get-attr event "text"))   # Enter sends: event.text is the message
```

`components/chat.ari` provides the `chat_window` unit and the functions behind it, `chat-ui-step`,
`chat-ui-send` and `chat-ui-more`. `components/room_users.ari` provides the users list as units,
and `room-users-step`, which builds its rows from `room-roster`. `conference_roster.ari` imports
both.
