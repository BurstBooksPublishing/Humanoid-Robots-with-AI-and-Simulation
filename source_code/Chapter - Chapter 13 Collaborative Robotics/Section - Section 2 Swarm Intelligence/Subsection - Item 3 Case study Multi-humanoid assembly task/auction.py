import asyncio
import time
import json
# mock comms API: send_broadcast(msg), recv_broadcast()
# each agent runs this loop independently.

AGENT_ID = "humanoid_A"        # agent identity
BID_TTL = 1.5                  # seconds until bid expires
HEARTBEAT = 0.2                # heartbeat interval

async def send_bid(task_id, cost):
    msg = {"type":"bid","agent":AGENT_ID,"task":task_id,"cost":cost,"t":time.time()}
    await send_broadcast(json.dumps(msg))   # non-blocking send

async def listen_loop(local_task_queue):
    while True:
        raw = await recv_broadcast()        # await incoming messages
        msg = json.loads(raw)
        if msg["type"] == "bid":
            # store incoming bid with timestamp for local auction
            local_task_queue[msg["task"]].append((msg["agent"], msg["cost"], msg["t"]))

async def auction_loop(local_task_queue):
    while True:
        for task, bids in list(local_task_queue.items()):
            # remove expired bids
            bids[:] = [b for b in bids if time.time()-b[2] < BID_TTL]
            if bids:
                # select lowest cost bidder
                winner = min(bids, key=lambda x:x[1])[0]
                # declare winner locally (no global cert required)
                await send_broadcast(json.dumps({"type":"award","task":task,"agent":winner}))
                del local_task_queue[task]
        await asyncio.sleep(HEARTBEAT)

# main starts listen and auction loops concurrently