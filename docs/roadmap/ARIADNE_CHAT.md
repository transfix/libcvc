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

## Bridged users

The users of another chat system, an NMDC hub's say, can be in a room beside its participants:
listed in its roster, posting to its chat and reading it, and messaging participants and being
messaged by them. The relay's program bridges them (arisrvx, `pycvc.ariadne`), and the relay is
their only writer. They have no camera, microphone, avatar or link.

**Who they are.** A bridged user is a user id on a system (`via`: a lowercase letter, then up to 15
lowercase letters and digits, such as `nmdc`). Its pid is `_<via>_<id>`, with every byte of the id
that is not an ASCII letter or digit written `_xx` (its hex): `Joe.R` on `nmdc` is `_nmdc_Joe_2eR`.
So the pid is a state name, which a DM channel is keyed by, and no two users share one. It begins
with `_`, and the room's write gate takes no write from a participant whose pid does, so no
connection can join as a bridged user.

**In the roster.** The relay writes them in a subtree of the room apart from the participants, so
presence, the area of interest and the holon never see them:

| path | written by | reaches |
|---|---|---|
| `bridge.rev` | relay | everyone (replicated): moves with every change below (the relay's clock in ms, or one past the last) |
| `bridge.count` | relay | everyone (replicated): how many are present |
| `bridge.users.<pid>.{present, uid, via, name, detail, e}` | relay | everyone (replicated) |

`e` is the relay's chat epoch when it wrote the user: what an earlier relay process wrote, which a
client that outlived it still holds, is not listed. Leaving is soft, as a participant's is: `present`
goes to `0` and the node stays; the same user coming back takes the same node. A room holds at most
10000 bridged users at once; an id is at most 128 bytes, a name 128 and a detail 256, each one line.

`(room-roster ROOM)` still lists the participants alone, so a document that subscribes to each
entry's media is unchanged. `(room-roster ROOM "bridged")` lists the bridged users and
`(room-roster ROOM "all")` the participants, then them. Every entry has `bridged` (`"0"` or `"1"`),
and a bridged one `via`, `uid` and `detail`, `video_off` `"1"` and empty link fields;
`(room-roster ROOM KIND LIMIT)` gives at most LIMIT bridged users, and `(room-bridged-count ROOM)`
counts them. The verbs read the users again only when `bridge.rev` moved. `(room-bridged-rev ROOM)`
is that rev, so a document rebuilds what it shows of them only then: `room_users.ari` keeps them in
rows of their own (`ROWS_b`, at most 100 shown, rebuilt at most every 30 ticks), with a name, their
system and a message button, and no camera window. A document never walks every bridged user in
the DSL: a hub's thousands would pass an action's step cap.

**In the chat.** A bridged user's message is a participant's message with `"via"`: stamped, kept
and pushed by the relay as any is, `from` its pid and `name` what the program gave:
`{"seq":3,"from":"_nmdc_zed","name":"Zed","t":...,"text":"hi","e":"<epoch>","via":"nmdc"}`. It is
under the same text limit (2000 bytes once cleaned), but not the rate limit: the program's system
has its own. `chat-messages` gives `via` (`""` for a participant's), and `chat.ari` shows the line
as `[HH:MM] Zed (nmdc): hi`.

- **The program posts** for a user to the room (it need not be present: a hub's own messages), or
  to a participant as a DM while the user is present. The DM is pushed to the participant alone,
  who holds it under the bridged pid like any conversation.
- **The program hears** every participant's room post, and each DM a participant sends a bridged
  user, on its script thread as a `room_chat` event (JSON: `{"room","to","to_uid","to_via","seq",
  "from","name","t","text"}`), never its own posts. A DM to a bridged user is refused ("they cannot
  get direct messages here") while the program has no hook for them, and once the user has left. A
  conversation ends once neither of its two is present, as between participants.

In Python (ARIADNE.md §9.8.7):

```python
host.room_presence(room, "Zed.Z", {"name": "Zed", "detail": "DC++ 0.868"}, via="nmdc")
host.room_chat_post(room, "hello", sender_id="Zed.Z", via="nmdc")             # to the room
host.room_chat_post(room, "psst", sender_id="Zed.Z", via="nmdc", to=pid)      # a DM
host.room_absent(room, "Zed.Z", via="nmdc")

@host.on_room_chat(room)
def heard(msg): ...          # msg.pid, msg.name, msg.text, msg.seq, msg.t

@host.on_room_dm(room)
def dm(msg): ...             # msg.to: the bridged user's id; msg.via; msg.pid: the sender
```

## The DSL surface

Room verbs, registered with `register_room_intrinsics`. `ROOM` is the `room-join` handle.
`CHANNEL` is `""` for the room, or a peer's pid.

- `(chat-post ROOM TEXT [TO])` returns `{ok}`, or `{ok #f error}` when the text is empty or too long.
- `(chat-info ROOM [CHANNEL])` returns `{rev count head first more loading error}`. It also makes the
  first-view and gap requests. `rev` changes whenever the messages held for the channel change.
- `(chat-messages ROOM [CHANNEL])` returns the held messages, oldest first, as dicts:
  `{seq from name text t clock mine via}` (`via`: a bridged user's system, else `""`).
- `(room-roster ROOM [KIND [LIMIT]])`, `(room-bridged-count ROOM)` and `(room-bridged-rev ROOM)`:
  the participants, the bridged users or both, how many are bridged, and the rev that moves with
  them (above).
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

`components/chat.ari` provides the `chat` window unit (and `chat_panel`) and the functions behind it, `chat-ui-step`,
`chat-ui-send` and `chat-ui-more`. `components/room_users.ari` provides the users list as units,
and `room-users-step`, which builds its rows from `room-roster` (and the bridged users' from
`(room-roster ROOM "bridged")`). `conference_roster.ari` imports
both.
