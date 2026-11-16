"""pycvc.ariadne's bridged users (docs/roadmap/ARIADNE_CHAT.md, "Bridged users"), under plain
python with a ServerHarness modelling the relay's room: a small bridge service, as an NMDC hub's
would be, puts its users in the room (host.room_presence / room_absent), posts for them
(host.room_chat_post) and hears the room (host.on_room_chat, host.on_room_dm). The harness's
clients are the room's participants:
  * bridged_pid matches the relay's (room_intrinsics.h): "_<via>_" and the id, escaped;
  * a bridged user is in the roster with "all" and "bridged", not by default, marked bridged, with
    its via, uid and detail; room_absent takes it out; a repeat updates its name;
  * a bridged post is kept in the room's history marked with its via, under the relay's text
    limit; a hub's own message posts without presence; a DM from a bridged user reaches its
    participant only, and needs presence and a participant who is there;
  * a participant's room post reaches on_room_chat (sync and async hooks, in order), a DM to a
    bridged user reaches on_room_dm with its id; the program's own posts are not handed back; a
    DM between two participants is not the program's; a DM to a bridged user is refused while
    no on_room_dm hook is there, and after it left;
  * what the relay would refuse raises: a bad via, a blank or long id, an empty or long post,
    another room than the harness's, unknown info keys.
"""

import asyncio
import sys

import pycvc

checks = 0


def check(cond, what):
    global checks
    if not cond:
        raise AssertionError(what)
    checks += 1


def raises(exc, fn, *args, **kwargs):
    try:
        fn(*args, **kwargs)
    except exc as e:
        return str(e)
    raise AssertionError(f"{getattr(fn, '__name__', fn)} did not raise {exc.__name__}")


if not hasattr(pycvc, "ariadne"):
    print("SKIP test_pycvc_ariadne_bridge: this pycvc ships no pycvc.ariadne")
    sys.exit(0)

from pycvc.ariadne import Host, RoomChatMessage, bridged_pid  # noqa: E402
from pycvc.ariadne.testing import ChatRefused, ServerHarness  # noqa: E402

# -- the pid, as the relay makes it ----------------------------------------------------------------

check(bridged_pid("Joe.R", via="nmdc") == "_nmdc_Joe_2eR", "an escaped '.'")
check(bridged_pid("a_b", via="nmdc") == "_nmdc_a_5fb", "an escaped '_'")
check(bridged_pid("é", via="nmdc") == "_nmdc__c3_a9", "UTF-8 bytes, each escaped")
check(bridged_pid("x") == "_bridge_x", "via defaults to bridge")
raises(ValueError, bridged_pid, "x", via="NMDC")
raises(ValueError, bridged_pid, "", via="nmdc")

# -- the service: a hub that echoes the room to its users and back ---------------------------------

host = Host()
check(host.room is None, "under plain python with no harness, no room is the relay's")
ROOM = "nmdc"
heard: list[RoomChatMessage] = []
heard_async: list[str] = []
dms: list[RoomChatMessage] = []
to_hub: list[str] = []  # what the bridge would send its hub's main chat


@host.on_room_chat(ROOM)
def on_chat(msg):
    heard.append(msg)
    to_hub.append(f"<{msg.name}> {msg.text}")


@host.on_room_chat(ROOM)
async def on_chat_later(msg):
    await asyncio.sleep(0)
    heard_async.append(msg.text)


def hub_join(nick, description=""):
    return host.room_presence(ROOM, nick, {"name": nick, "detail": description}, via="nmdc")


harness = ServerHarness(host, room=ROOM)
check(host.room == ROOM, "the harness names the room")
alice = harness.connect(pid="alice", name="Alice")
bob = harness.connect(pid="bob", name="Bob")

# -- bridged users in the roster -------------------------------------------------------------------

zed = hub_join("Zed.Z", "<DC++ V:0.868>")
check(zed == "_nmdc_Zed_2eZ", "room_presence returns the pid")
hub_join("ann")
check([e["pid"] for e in alice.roster()] == ["alice", "bob"], "the default: participants only")
everyone = alice.roster("all")
check([e["pid"] for e in everyone] == ["alice", "bob", zed, "_nmdc_ann"],
      "all: the participants, then the bridged, in pid order (bytes: 'Z' before 'a')")
z = everyone[2]
check(z["bridged"] == "1" and z["via"] == "nmdc" and z["uid"] == "Zed.Z" and z["name"] == "Zed.Z"
      and z["detail"] == "<DC++ V:0.868>", f"a bridged entry: {z}")
check(everyone[0]["bridged"] == "0", "a participant is not bridged")
check([e["pid"] for e in alice.roster("bridged")] == [zed, "_nmdc_ann"], "bridged only")
host.room_presence(ROOM, "Zed.Z", {"name": "Zed"}, via="nmdc")
check(alice.roster("bridged")[0]["name"] == "Zed", "a repeat updates the name")
check([u["user_id"] for u in host.room_bridged()] == ["Zed.Z", "ann"], "the program's own list")

# -- bridged posts ---------------------------------------------------------------------------------

host.room_chat_post(ROOM, "hello from the hub", sender_id="Zed.Z", via="nmdc")
host.room_chat_post(ROOM, "Welcome to the hub", sender_id="Hub-Security", via="nmdc",
                    name="Hub-Security")
msgs = bob.chat_messages()
check([m["text"] for m in msgs] == ["hello from the hub", "Welcome to the hub"],
      "kept in the room's history")
check(msgs[0]["from"] == zed and msgs[0]["name"] == "Zed" and msgs[0]["via"] == "nmdc",
      f"stamped as the bridged user, with its via: {msgs[0]}")
check(msgs[1]["from"] == "_nmdc_Hub_2dSecurity", "a sender need not be present to post")
check(heard == [] and heard_async == [], "the program does not hear its own posts")
check(harness.room_messages()[0]["via"] == "nmdc", "harness.room_messages")

host.room_chat_post(ROOM, "psst alice", sender_id="Zed.Z", via="nmdc", to="alice")
check([m["text"] for m in alice.chat_messages(zed)] == ["psst alice"], "a DM reaches alice")
check(bob.chat_messages(zed) == [], "and no one else")
why = raises(ValueError, host.room_chat_post, ROOM, "hi", sender_id="nobody", via="nmdc",
             to="alice")
check("only while present" in why, why)
why = raises(ValueError, host.room_chat_post, ROOM, "hi", sender_id="Zed.Z", via="nmdc", to="carol")
check("not in the room" in why, why)

# -- what participants post -----------------------------------------------------------------------

m = alice.chat_post("hi hub")
check(m["from"] == "alice" and m["seq"] == 2, f"kept after the bridged posts: {m}")
check(len(heard) == 1 and heard[0].pid == "alice" and heard[0].name == "Alice"
      and heard[0].text == "hi hub" and heard[0].to is None and heard[0].room == ROOM
      and heard[0].seq == 2, f"on_room_chat: {heard}")
check(heard_async == ["hi hub"], "an async hook ran to the end")
check(to_hub == ["<Alice> hi hub"], "what the bridge sends its hub")
bob.chat_post("two\n  lines  \n")
check(heard[-1].text == "two\n  lines", "cleaned as the relay cleans it")

why = raises(ChatRefused, alice.chat_post, "hello zed", zed)
check("cannot get direct messages" in why, f"no on_room_dm hook yet: {why}")


@host.on_room_dm(ROOM)
def on_dm(msg):
    dms.append(msg)


alice.chat_post("hello zed", zed)
check(len(dms) == 1 and dms[0].to == "Zed.Z" and dms[0].to_pid == zed and dms[0].via == "nmdc"
      and dms[0].pid == "alice" and dms[0].text == "hello zed", f"on_room_dm: {dms}")
check([x["text"] for x in alice.chat_messages(zed)] == ["psst alice", "hello zed"],
      "one conversation, both ways")
n = len(heard)
alice.chat_post("just us", "bob")
check(len(heard) == n and len(dms) == 1, "a DM between participants is not the program's")
check(bob.chat_messages("alice")[0]["text"] == "just us", "but bob has it")

host.room_absent(ROOM, "Zed.Z", via="nmdc")
check([e["pid"] for e in alice.roster("bridged")] == ["_nmdc_ann"], "room_absent")
host.room_absent(ROOM, "Zed.Z", via="nmdc")  # gone already: nothing
why = raises(ChatRefused, alice.chat_post, "zed?", zed)
check("not in the room" in why, why)

# -- what the relay would refuse ------------------------------------------------------------------

raises(ValueError, host.room_presence, ROOM, "x", {"name": "x"}, via="Nmdc")
raises(ValueError, host.room_presence, ROOM, "  ", via="nmdc")
raises(ValueError, host.room_presence, ROOM, "x" * 129, via="nmdc")
raises(ValueError, host.room_presence, ROOM, "x", {"nick": "x"}, via="nmdc")
raises(ValueError, host.room_presence, ROOM, "x", {"name": "n" * 129}, via="nmdc")
raises(TypeError, host.room_presence, ROOM, 7, via="nmdc")
raises(ValueError, host.room_chat_post, ROOM, " \n ", sender_id="ann", via="nmdc")
why = raises(ValueError, host.room_chat_post, ROOM, "x" * 2001, sender_id="ann", via="nmdc")
check("2000 bytes" in why, why)
raises(TypeError, host.room_chat_post, ROOM, b"bytes", sender_id="ann", via="nmdc")
why = raises(ValueError, host.room_chat_post, "standup", "hi", sender_id="ann", via="nmdc")
check("'nmdc'" in why, f"another room: {why}")
raises(ChatRefused, alice.chat_post, "")
raises(ChatRefused, alice.chat_post, "me", "alice")

# A participant who leaves is out of the roster and posts nothing.
bob.leave()
check([e["pid"] for e in alice.roster()] == ["alice"], "bob left")
raises(ChatRefused, bob.chat_post, "still here?")

print(f"test_pycvc_ariadne_bridge: {checks} checks passed")
