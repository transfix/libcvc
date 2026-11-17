"""The program beside the relay for the bridged-users ctests (arisrvx_bridge_arigl_*): a stand-in
for another chat system's bridge, as an NMDC hub's would be (docs/roadmap/ARIADNE_CHAT.md,
"Bridged users"). It puts two of the system's users in the relay's room and welcomes the room as
one of them; it echoes each participant's room post back as that user, answers a direct message
to it, and on "bye" takes the other user out of the room and says so."""

from pycvc.ariadne import Host

host = Host()
ROOM = host.room  # the relay's --room

host.room_presence(ROOM, "Zed.Z", {"name": "Zed", "detail": "on the hub"}, via="nmdc")
host.room_presence(ROOM, "ann", via="nmdc")
host.room_chat_post(ROOM, "welcome from the hub", sender_id="Zed.Z", via="nmdc")


@host.on_room_chat(ROOM)
def echo(msg):
    if msg.text == "bye":
        host.room_absent(ROOM, "ann", via="nmdc")
        host.room_chat_post(ROOM, "ann left the hub", sender_id="Hub", name="Hub", via="nmdc")
        return
    host.room_chat_post(ROOM, f"echo {msg.name}: {msg.text}", sender_id="Zed.Z", via="nmdc")


@host.on_room_dm(ROOM)
async def answer(msg):
    host.room_chat_post(ROOM, f"pong {msg.text}", sender_id=msg.to, via=msg.via, to=msg.pid)
