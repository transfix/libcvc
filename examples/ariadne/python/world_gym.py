"""world_gym.py — a policy of yours drives the lab's vehicles in world_gym.ari, headless, as a gym
(pycvc.WorldGym; ARIADNE_WORLDS.md section 14, L4).

    python world_gym.py                               # 4 Humvees on the flat square, 3 episodes
    python world_gym.py --world generated --agents 6  # the generated world (its streets, buildings)
    python world_gym.py --navigator                   # nav@1 drives (the baseline to beat)
    python world_gym.py --log episodes.csv

Each episode resets the world (the same seed, the same episode), spawns the agents at the
document's spawn points (or free points of the grid), and steps them 0.05 s at a time until every
agent is at its goal, one leaves the world (off its edge, or through its ground), or --steps run
out. The policy below is the simplest that works: steer at the goal, brake when the ray ahead is
short. The reward is the gym's (progress toward the goal, less one per tick touching something,
plus ten on arrival) unless you pass your own to set_reward(). To watch the same world drawn, open
world_gym.ari in arigl.
"""

import argparse
import math
import os
import sys

import numpy as np

import pycvc

HERE = os.path.dirname(os.path.abspath(__file__))


def policy(obs):
    """Steer toward the goal; ease off and brake when the ray straight ahead is short."""
    to_goal = np.arctan2(obs["goal_y"] - obs["y"], obs["goal_x"] - obs["x"])
    err = (to_goal - obs["yaw"] + math.pi) % (2.0 * math.pi) - math.pi
    steer = np.clip(1.5 * err, -obs["max_steer"], obs["max_steer"])
    ahead = obs["ranges"][:, len(obs["ray_angles"]) // 2]  # the ray at 0 rad
    want = np.minimum(8.0, 0.5 * ahead) * np.clip(np.cos(err), 0.2, 1.0)
    want = np.where(obs["goal_dist"] < 15.0, np.minimum(want, 0.3 * obs["goal_dist"]), want)
    accel = np.clip(1.5 * (want - obs["speed"]), -obs["max_decel"], obs["max_accel"])
    return steer, accel


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--doc", default=os.path.join(HERE, "world_gym.ari"))
    p.add_argument("--world", default="flat", help="flat, terrain or generated")
    p.add_argument("--agents", type=int, default=4)
    p.add_argument("--episodes", type=int, default=3)
    p.add_argument("--steps", type=int, default=1200, help="per episode (0.05 s each)")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--navigator", action="store_true", help="nav@1 drives instead of policy()")
    p.add_argument("--log", default=None, help="write every step of every episode (.csv)")
    a = p.parse_args(argv)

    gym = pycvc.WorldGym(a.doc, world=a.world, count=a.agents, max_steps=a.steps)
    for w in gym.take_warnings():
        print("world_gym:", w, file=sys.stderr)
    for ep in range(a.episodes):
        obs = gym.reset(seed=a.seed + ep)
        while True:
            if a.navigator:
                steer = accel = np.full(obs["n"], np.nan)
            else:
                steer, accel = policy(obs)
            obs, reward, terminated, truncated, info = gym.step(steer, accel)
            if terminated or truncated:
                break
        log = gym.log()
        mine = log["episode"] == obs["episode"]
        print(f"episode {obs['episode']}: {obs['time']:.1f} s, "
              f"{int(obs['reached'].sum())}/{obs['n']} at their goals, "
              f"return {info['return']:.1f}, "
              f"{int(log['collided'][mine].sum())} agent-ticks touching"
              + (" (an agent left the world)" if obs["out"].any()
                 else " (all arrived)" if terminated else ""))
    if a.log:
        gym.save_log(a.log)
        print(f"world_gym: the log in {a.log}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
