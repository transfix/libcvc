"""The program beside the relay for ctest arisrvx_clients_program (ARISRV_IDENTITY.md 2.8, ID4c):
connections the program serves, as an NMDC hub's users would be, in the relay's client list
through host.clients. The client document opens them through these exports (a hub would from its
listener's hooks), and an admin's server.kick and server.ban reach them through the closer."""

from pycvc.ariadne import ClientRefused, Host, export

host = Host()
closed = []  # what the closer was told, [key, reason, kind], in order
ids = {}  # key -> the client id open_engine returned


@host.clients.set_closer
async def close(key, reason, kind):
    closed.append([key, reason, kind])
    await host.clients.close_engine(key)  # as a hub's on_close would, once it closed the socket


@export(clients=True)
async def open_nmdc(ctx, key, address, nick):
    cid = await host.clients.open_engine(key, transport="nmdc", address=address, kind="nmdc",
                                         version="<DC++ V:0.868>",
                                         capabilities=["chat", "chat.dm"],
                                         geo={"country": "US", "city": "Austin"})
    ids[key] = cid
    await host.clients.name_engine(key, nick)
    # Its $MyINFO said it shares files.
    await host.clients.update_engine(key, kind="nmdc", version="<DC++ V:0.868>",
                                     capabilities=["chat", "chat.dm", "files.share"])
    return cid


@export(clients=True)
async def try_open(ctx, key, address):
    try:
        cid = await host.clients.open_engine(key, transport="nmdc", address=address, kind="nmdc")
    except ClientRefused as e:
        return e.code
    ids[key] = cid
    return cid


@export(clients=True)
async def try_name(ctx, key, nick):
    try:
        await host.clients.name_engine(key, nick)
    except ClientRefused as e:
        return e.code
    return "ok"


@export(clients=True)
async def close_twice(ctx, key):
    return [await host.clients.close_engine(key), await host.clients.close_engine(key)]


@export(clients=True)
def client_of(ctx, key):
    return ids[key]


@export(clients=True)
def closed_log(ctx):
    return closed


host.run()
