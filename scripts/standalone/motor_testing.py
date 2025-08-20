# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""This script demonstrates how to spawn a cart-pole and interact with it.

.. code-block:: bash

    # Usage
    ./isaaclab.sh -p scripts/tutorials/01_assets/run_articulation.py

"""

"""Launch Isaac Sim Simulator first."""


import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on spawning and interacting with an articulation.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch

import isaacsim.core.utils.prims as prim_utils

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.sim import SimulationContext

##
# Pre-defined configs
##
from mistletoe.robots.mistletoe_v4_fixed import MISTLETOE_CFG

# trapezoid traj configs

import math

import csv

VELOCITY_LIMIT = 0.5 * 2 * math.pi
ACCELERATION_LIMIT = 2 * 2 * math.pi 

def trapezoidal_trajectory_tensor(x0, xf, v_max, a, dt=0.01, device='cpu'):
    """
    Generates a trapezoidal velocity profile as PyTorch tensors.

    Parameters:
        x0, xf : float
            Initial and final positions
        v_max : float
            Maximum velocity
        a : float
            Maximum acceleration
        dt : float
            Time step
        device : str
            PyTorch device ('cpu' or 'cuda')
    
    Returns:
        t, x, v, acc : torch.Tensor
            Time, position, velocity, acceleration tensors
    """
    D = xf - x0

    t_acc = v_max / a
    d_acc = 0.5 * a * t_acc**2

    if 2 * d_acc > abs(D):
        # Triangular profile
        t_acc = torch.sqrt(torch.tensor(abs(D)/a))
        t_const = 0
        v_peak = a * t_acc
    else:
        t_const = (abs(D) - 2 * d_acc) / v_max
        v_peak = v_max

    t_total = 2 * t_acc + t_const

    t = torch.arange(0, t_total + dt, dt, device=device)
    x = torch.zeros_like(t, device=device)
    v = torch.zeros_like(t, device=device)
    acc = torch.zeros_like(t, device=device)

    # Acceleration phase
    mask_acc = t < t_acc
    x[mask_acc] = x0 + 0.5 * a * t[mask_acc]**2
    v[mask_acc] = a * t[mask_acc]
    acc[mask_acc] = a

    # Constant velocity phase
    mask_const = (t >= t_acc) & (t < t_acc + t_const)
    x[mask_const] = x0 + d_acc + v_peak * (t[mask_const] - t_acc)
    v[mask_const] = v_peak
    acc[mask_const] = 0

    # Deceleration phase
    mask_dec = t >= t_acc + t_const
    t_dec = t[mask_dec] - t_acc - t_const
    x[mask_dec] = xf - 0.5 * a * (t_total - t[mask_dec])**2
    v[mask_dec] = v_peak - a * t_dec
    acc[mask_dec] = -a

    # Handle negative displacement
    if D < 0:
        x = x0 - (x - x0)
        v = -v
        acc = -acc

    return t, x, v, acc

def design_scene() -> tuple[dict, list[list[float]]]:
    """Designs the scene."""
    # Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg, translation=[0,0,-1])
    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    # Create separate groups called "Origin1", "Origin2"
    # Each group will have a robot in it
    origins = [[0.0, 0.0, 0]]
    # Origin 1
    prim_utils.create_prim("/World/Origin1", "Xform", translation=origins[0])
    # Articulation
    mistletoe_cfg = MISTLETOE_CFG.copy()
    mistletoe_cfg.prim_path = "/World/Origin.*/Robot"
    mistletoe = Articulation(cfg=mistletoe_cfg)

    # return the scene information
    scene_entities = {"mistletoe": mistletoe}
    return scene_entities, origins


def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, Articulation], origins: torch.Tensor):
    """Runs the simulation loop."""
    # Extract scene entities
    # note: we only do this here for readability. In general, it is better to access the entities directly from
    #   the dictionary. This dictionary is replaced by the InteractiveScene class in the next tutorial.
    robot = entities["mistletoe"]
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    count = 0
    # Simulation loop
    
    done = False

    t, x, v, a = trapezoidal_trajectory_tensor(0, math.pi/2, VELOCITY_LIMIT, ACCELERATION_LIMIT, device=args_cli.device, dt=sim_dt)
    with open("joint_positions.csv", mode="w", newline="") as f:
        writer = csv.writer(f)
        while simulation_app.is_running():
            if count > len(x) - 2:
                # # reset counter

                for i in range(50):
                    robot.set_joint_position_target(x[count - 1])
                    # -- write data to sim
                    robot.write_data_to_sim()
                    joint_pos = robot.data.joint_pos[0]
                    joint_vel = robot.data.joint_vel[0]
                    # print(f'real joint pos {joint_pos[1]}')

                    joint_vel = joint_vel.tolist()
                    joint_vel.append(v[count].item())

                    writer.writerow(joint_pos.tolist() + joint_vel)
                    # Perform step
                    sim.step()
                    # Increment counter
                    # count += 1
                    # Update buffers
                    robot.update(sim_dt)
                # count = 0
                # # reset the scene entities
                # # root state
                # # we offset the root state by the origin since the states are written in simulation world frame
                # # if this is not done, then the robots will be spawned at the (0, 0, 0) of the simulation world
                # root_state = robot.data.default_root_state.clone()
                # root_state[:, :3] += origins
                # robot.write_root_pose_to_sim(root_state[:, :7])
                # robot.write_root_velocity_to_sim(root_state[:, 7:])
                # # set joint positions with some noise
                # joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
                # joint_pos += torch.rand_like(joint_pos) * 0.1
                # robot.write_joint_state_to_sim(joint_pos, joint_vel)
                # # clear internal buffers
                # robot.reset()
                # print("[INFO]: Resetting robot state...")
                f.close()
                done = True
                # simulation_app.close()\

            # Apply random action
            # -- generate random joint efforts
            # print(f'target {x[count]}')
            

            if not done:
            # -- apply action to the robot
                robot.set_joint_effort_target(x[count])
                # -- write data to sim
                joint_pos = robot.data.joint_pos[0]
                joint_vel = robot.data.joint_vel[0]
                # print(f'real joint pos {joint_pos[1]}')

                joint_vel = joint_vel.tolist()
                joint_vel.append(v[count].item())

                writer.writerow(joint_pos.tolist() + joint_vel)

            robot.write_data_to_sim()
            # Perform step
            sim.step()
            # Increment counter
            count += 1
            # Update buffers
            robot.update(sim_dt)
            print(sim_dt)


def main():
    """Main function."""
    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([2.5, 0.0, 4.0], [0.0, 0.0, 2.0])
    # Design scene
    scene_entities, scene_origins = design_scene()
    scene_origins = torch.tensor(scene_origins, device=sim.device)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene_entities, scene_origins)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()