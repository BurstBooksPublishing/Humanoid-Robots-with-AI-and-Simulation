import time
import torch
from queue import Queue
from threading import Thread

BATCH_SIZE = 32
in_queue = Queue()   # observations pushed by simulators/robots
out_queue = Queue()  # control outputs returned

model = MyPolicyModel().cuda()  # pretrained PyTorch model

def batch_worker():
    while True:
        batch = []
        ids = []
        # gather up to BATCH_SIZE items
        while len(batch) < BATCH_SIZE:
            obs_id, obs = in_queue.get()  # blocking get
            if obs is None:
                return  # shutdown signal
            ids.append(obs_id)
            batch.append(obs)
        x = torch.stack(batch).cuda()          # batch to GPU
        with torch.no_grad():
            actions = model(x)                 # single forward pass
        for idx, act in zip(ids, actions.cpu()):
            out_queue.put((idx, act))         # return per-agent outputs

# start background batching thread
t = Thread(target=batch_worker, daemon=True)
t.start()

# simulator loop pushes observations (example comment)
# in_queue.put((agent_id, observation_tensor))