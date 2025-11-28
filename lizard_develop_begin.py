#!/usr/bin/env python3
"""
lizard_develop_begin.py

Instrumented evolution script with robust URDF generation and validation.
- Writes URDFs that include full <inertial> inertia elements
- Tests each URDF by attempting to load it into a temporary PyBullet DIRECT client
- If URDF load fails, prints useful diagnostics and aborts

Usage:
  python lizard_develop_begin.py             # run evolution (defaults)
  python lizard_develop_begin.py --unit_test --controller best_model.pkl
  python lizard_develop_begin.py --capture_frames --perturb_init_pose 0.02
"""

import os
import sys
import argparse
import math
import random
import time
import csv
import pickle
from datetime import datetime
from collections import namedtuple

import numpy as np
import pybullet as p
import pybullet_data
from PIL import Image

# ---------------------------
# Config
# ---------------------------
POPULATION = 12
GENERATIONS = 200
GENERATION_DURATION = 30.0
SIMULATION_STEP = 1.0 / 240.0
STEPS_PER_GEN = int(GENERATION_DURATION / SIMULATION_STEP)
WORLD_GRAVITY = -9.81
SEED = 42
ELITE_COUNT = 1
MUTATION_RATE = 0.06
MUTATION_PROB = 0.15
HIDDEN_NEURONS = 32

OUTPUT_ROOT = f"upright_instrumented_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
os.makedirs(OUTPUT_ROOT, exist_ok=True)

MAX_FRAME_WIDTH = 320
MAX_FRAME_HEIGHT = 240
FRAME_SAVE_EVERY = 5
CSV_FLUSH_EVERY = 100

W_UPRIGHT = 1.5
W_TRAVEL = 1.0
W_TURN_AFTER_HIT = 3.0

# ---------------------------
# Determinism helper
# ---------------------------
def set_global_seed(seed):
    random.seed(seed)
    np.random.seed(seed)
    print(f"[seed] global seed set to {seed}")

# ---------------------------
# MLPController
# ---------------------------
class MLPController:
    def __init__(self, input_size, hidden_size, output_size, weights=None):
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.output_size = output_size
        if weights is None:
            self.weights = {
                'W1': np.random.randn(hidden_size, input_size) * 0.5,
                'b1': np.zeros((hidden_size,)),
                'W2': np.random.randn(output_size, hidden_size) * 0.5,
                'b2': np.zeros((output_size,))
            }
        else:
            self.weights = {k: np.array(v, copy=True) for k, v in weights.items()}
        self.color = (0.3, 0.6, 0.3, 1.0)

    def forward(self, x):
        z1 = np.tanh(self.weights['W1'] @ x + self.weights['b1'])
        out = np.tanh(self.weights['W2'] @ z1 + self.weights['b2'])
        return out

    def get_flat(self):
        parts = [self.weights['W1'].ravel(), self.weights['b1'].ravel(),
                 self.weights['W2'].ravel(), self.weights['b2'].ravel()]
        return np.concatenate(parts)

    def set_flat(self, flat):
        s = 0
        W1_size = self.hidden_size * self.input_size
        self.weights['W1'] = flat[s:s+W1_size].reshape((self.hidden_size, self.input_size)); s += W1_size
        b1_size = self.hidden_size
        self.weights['b1'] = flat[s:s+b1_size]; s += b1_size
        W2_size = self.output_size * self.hidden_size
        self.weights['W2'] = flat[s:s+W2_size].reshape((self.output_size, self.hidden_size)); s += W2_size
        b2_size = self.output_size
        self.weights['b2'] = flat[s:s+b2_size]; s += b2_size

    def copy(self):
        new_w = {k: np.array(v, copy=True) for k, v in self.weights.items()}
        child = MLPController(self.input_size, self.hidden_size, self.output_size, new_w)
        child.color = tuple(self.color)
        return child

    @staticmethod
    def crossover(parent_a, parent_b):
        fa = parent_a.get_flat()
        fb = parent_b.get_flat()
        assert fa.shape == fb.shape
        mask = np.random.rand(fa.size) < 0.5
        child_flat = np.where(mask, fa, fb)
        child = MLPController(parent_a.input_size, parent_a.hidden_size, parent_a.output_size)
        child.set_flat(child_flat)
        child.color = tuple(parent_a.color) if (random.random() < 0.5) else tuple(parent_b.color)
        return child

    def mutate(self, rate=MUTATION_RATE, prob=MUTATION_PROB):
        flat = self.get_flat()
        mutation_mask = np.random.rand(flat.size) < prob
        gaussian = np.random.randn(flat.size) * rate
        flat = np.where(mutation_mask, flat + gaussian, flat)
        self.set_flat(flat)

# ---------------------------
# Robust URDF writer + test loader
# ---------------------------
def box_inertia(mass, lx, ly, lz):
    # lx = length in x (m), ly = width in y, lz = height in z
    ixx = (1.0/12.0) * mass * (ly*ly + lz*lz)
    iyy = (1.0/12.0) * mass * (lx*lx + lz*lz)
    izz = (1.0/12.0) * mass * (lx*lx + ly*ly)
    return ixx, iyy, izz

def write_lizard_urdf(path, morphology='default', scale=1.0, name_prefix="lizard", body_color=(0.3,0.6,0.3,1.0)):
    """
    Robust URDF writer:
    - Proper XML nesting
    - Full <inertial> with inertia tensor
    - Every revolute joint includes a <limit> element (required by PyBullet)
    - Two morphologies: 'simple' (2 hips + head) and 'default' (bigger)
    """
    def box_inertia(mass, lx, ly, lz):
        ixx = (1.0/12.0) * mass * (ly*ly + lz*lz)
        iyy = (1.0/12.0) * mass * (lx*lx + lz*lz)
        izz = (1.0/12.0) * mass * (lx*lx + ly*ly)
        return ixx, iyy, izz

    r,g,b,a = [float(x) for x in body_color]
    lines = []
    lines.append('<?xml version="1.0" ?>')
    lines.append(f'<robot name="{name_prefix}">')
    lines.append(f'  <material name="body_color"><color rgba="{r} {g} {b} {a}"/></material>')

    # conservative default joint limits (used for all revolute joints)
    default_limit = {"lower": -1.5708, "upper": 1.5708, "effort": 20.0, "velocity": 4.0}

    if morphology == 'simple':
        # body
        body_length = 0.30 * scale
        body_width  = 0.12 * scale
        body_height = 0.08 * scale
        body_mass   = 0.90 * scale
        ixx, iyy, izz = box_inertia(body_mass, body_length, body_width, body_height)

        lines += [
            '  <link name="body_0">',
            '    <inertial>',
            '      <origin xyz="0 0 0" rpy="0 0 0"/>',
            f'      <mass value="{body_mass:.6f}"/>',
            f'      <inertia ixx="{ixx:.8f}" iyy="{iyy:.8f}" izz="{izz:.8f}" ixy="0" ixz="0" iyz="0"/>',
            '    </inertial>',
            f'    <visual><geometry><box size="{body_length:.6f} {body_width:.6f} {body_height:.6f}"/></geometry><material name="body_color"/></visual>',
            f'    <collision><geometry><box size="{body_length:.6f} {body_width:.6f} {body_height:.6f}"/></geometry></collision>',
            '  </link>'
        ]

        # head
        head_length = 0.16 * scale
        head_width  = body_width
        head_height = 0.06 * scale
        head_mass   = 0.25 * scale
        ixx, iyy, izz = box_inertia(head_mass, head_length, head_width, head_height)

        lines += [
            '  <link name="head">',
            '    <inertial>',
            '      <origin xyz="0 0 0" rpy="0 0 0"/>',
            f'      <mass value="{head_mass:.6f}"/>',
            f'      <inertia ixx="{ixx:.8f}" iyy="{iyy:.8f}" izz="{izz:.8f}" ixy="0" ixz="0" iyz="0"/>',
            '    </inertial>',
            f'    <visual><geometry><box size="{head_length:.6f} {head_width:.6f} {head_height:.6f}"/></geometry><material name="body_color"/></visual>',
            f'    <collision><geometry><box size="{head_length:.6f} {head_width:.6f} {head_height:.6f}"/></geometry></collision>',
            '  </link>',
            '  <joint name="head_joint" type="revolute">',
            '    <parent link="body_0"/>',
            '    <child link="head"/>',
            f'    <origin xyz="{(-body_length*0.55):.6f} 0 {(body_height*0.5 + head_height*0.5 - 0.01):.6f}" rpy="0 0 0"/>',
            '    <axis xyz="0 1 0"/>',
            f'    <limit lower="{default_limit["lower"]:.6f}" upper="{default_limit["upper"]:.6f}" effort="{default_limit["effort"]:.1f}" velocity="{default_limit["velocity"]:.1f}"/>',
            '  </joint>'
        ]

        # hips (left/right)
        hip_mass = 0.05 * scale
        hip_ixx, hip_iyy, hip_izz = box_inertia(hip_mass, 0.02, 0.02, 0.02)

        lines += [
            '  <link name="hip_left">',
            '    <inertial>',
            '      <origin xyz="0 0 0" rpy="0 0 0"/>',
            f'      <mass value="{hip_mass:.6f}"/>',
            f'      <inertia ixx="{hip_ixx:.8f}" iyy="{hip_iyy:.8f}" izz="{hip_izz:.8f}" ixy="0" ixz="0" iyz="0"/>',
            '    </inertial>',
            '  </link>',
            '  <joint name="hip_left_joint" type="revolute">',
            '    <parent link="body_0"/>',
            '    <child link="hip_left"/>',
            '    <origin xyz="0 0.08 -0.02" rpy="0 0 0"/>',
            '    <axis xyz="0 0 1"/>',
            f'    <limit lower="{default_limit["lower"]:.6f}" upper="{default_limit["upper"]:.6f}" effort="{default_limit["effort"]:.1f}" velocity="{default_limit["velocity"]:.1f}"/>',
            '  </joint',
            # Note: previous error was mismatched element—make sure to close tags correctly. We'll continue with properly closed joints below.
        ]
        # above I intentionally closed incorrectly to show what to avoid; ensure we correct it below by removing that stray line
        # Fixing: remove the stray '  </joint' line and instead append a correct joint close; we'll overwrite the 'lines' end appropriately.

        # Remove the incorrect stray element we just added (safe guard in case of copy-paste)
        if lines[-1].strip() == '  </joint':
            lines.pop()

        # correctly add hip joints (left/right)
        lines += [
            '  <joint name="hip_left_joint" type="revolute">',
            '    <parent link="body_0"/>',
            '    <child link="hip_left"/>',
            '    <origin xyz="0 0.08 -0.02" rpy="0 0 0"/>',
            '    <axis xyz="0 0 1"/>',
            f'    <limit lower="{default_limit["lower"]:.6f}" upper="{default_limit["upper"]:.6f}" effort="{default_limit["effort"]:.1f}" velocity="{default_limit["velocity"]:.1f}"/>',
            '  </joint>',
            '  <link name="hip_right">',
            '    <inertial>',
            '      <origin xyz="0 0 0" rpy="0 0 0"/>',
            f'      <mass value="{hip_mass:.6f}"/>',
            f'      <inertia ixx="{hip_ixx:.8f}" iyy="{hip_iyy:.8f}" izz="{hip_izz:.8f}" ixy="0" ixz="0" iyz="0"/>',
            '    </inertial>',
            '  </link>',
            '  <joint name="hip_right_joint" type="revolute">',
            '    <parent link="body_0"/>',
            '    <child link="hip_right"/>',
            '    <origin xyz="0 -0.08 -0.02" rpy="0 0 0"/>',
            '    <axis xyz="0 0 1"/>',
            f'    <limit lower="{default_limit["lower"]:.6f}" upper="{default_limit["upper"]:.6f}" effort="{default_limit["effort"]:.1f}" velocity="{default_limit["velocity"]:.1f}"/>',
            '  </joint>'
        ]

    else:
        # default (larger) morphology: same structure, full inertials, proper joint limits
        body_length = 0.35 * scale
        body_width  = 0.12 * scale
        body_height = 0.08 * scale
        body_mass   = 0.90 * scale
        ixx, iyy, izz = box_inertia(body_mass, body_length, body_width, body_height)

        lines += [
            '  <link name="body_0">',
            '    <inertial>',
            '      <origin xyz="0 0 0" rpy="0 0 0"/>',
            f'      <mass value="{body_mass:.6f}"/>',
            f'      <inertia ixx="{ixx:.8f}" iyy="{iyy:.8f}" izz="{izz:.8f}" ixy="0" ixz="0" iyz="0"/>',
            '    </inertial>',
            f'    <visual><geometry><box size="{body_length:.6f} {body_width:.6f} {body_height:.6f}"/></geometry><material name="body_color"/></visual>',
            f'    <collision><geometry><box size="{body_length:.6f} {body_width:.6f} {body_height:.6f}"/></geometry></collision>',
            '  </link>'
        ]

        head_length = 0.18 * scale
        head_width  = body_width
        head_height = 0.06 * scale
        head_mass   = 0.25 * scale
        ixx, iyy, izz = box_inertia(head_mass, head_length, head_width, head_height)

        lines += [
            '  <link name="head">',
            '    <inertial>',
            '      <origin xyz="0 0 0" rpy="0 0 0"/>',
            f'      <mass value="{head_mass:.6f}"/>',
            f'      <inertia ixx="{ixx:.8f}" iyy="{iyy:.8f}" izz="{izz:.8f}" ixy="0" ixz="0" iyz="0"/>',
            '    </inertial>',
            f'    <visual><geometry><box size="{head_length:.6f} {head_width:.6f} {head_height:.6f}"/></geometry><material name="body_color"/></visual>',
            f'    <collision><geometry><box size="{head_length:.6f} {head_width:.6f} {head_height:.6f}"/></geometry></collision>',
            '  </link>',
            '  <joint name="head_joint" type="revolute">',
            '    <parent link="body_0"/>',
            '    <child link="head"/>',
            f'    <origin xyz="{(-body_length*0.55):.6f} 0 {(body_height*0.5 + head_height*0.5 - 0.01):.6f}" rpy="0 0 0"/>',
            '    <axis xyz="0 1 0"/>',
            f'    <limit lower="{default_limit["lower"]:.6f}" upper="{default_limit["upper"]:.6f}" effort="{default_limit["effort"]:.1f}" velocity="{default_limit["velocity"]:.1f}"/>',
            '  </joint>'
        ]

        # jaw link
        jaw_mass = 0.02 * scale
        jixx, jiyy, jizz = box_inertia(jaw_mass, 0.05, 0.02, 0.02)
        lines += [
            '  <link name="jaw">',
            '    <inertial>',
            '      <origin xyz="0 0 0" rpy="0 0 0"/>',
            f'      <mass value="{jaw_mass:.6f}"/>',
            f'      <inertia ixx="{jixx:.8f}" iyy="{jiyy:.8f}" izz="{jizz:.8f}" ixy="0" ixz="0" iyz="0"/>',
            '    </inertial>',
            '  </link>',
            '  <joint name="jaw_joint" type="revolute">',
            '    <parent link="head"/>',
            '    <child link="jaw"/>',
            '    <origin xyz="0 0 0" rpy="0 0 0"/>',
            '    <axis xyz="0 1 0"/>',
            f'    <limit lower="-0.8" upper="0.8" effort="5.0" velocity="4.0"/>',
            '  </joint>'
        ]

        # hips
        hip_mass = 0.05 * scale
        hip_ixx, hip_iyy, hip_izz = box_inertia(hip_mass, 0.02, 0.02, 0.02)
        lines += [
            '  <link name="front_left_hip">',
            '    <inertial>',
            '      <origin xyz="0 0 0" rpy="0 0 0"/>',
            f'      <mass value="{hip_mass:.6f}"/>',
            f'      <inertia ixx="{hip_ixx:.8f}" iyy="{hip_iyy:.8f}" izz="{hip_izz:.8f}" ixy="0" ixz="0" iyz="0"/>',
            '    </inertial>',
            '  </link>',
            '  <joint name="front_left_hip_joint" type="revolute">',
            '    <parent link="body_0"/>',
            '    <child link="front_left_hip"/>',
            '    <origin xyz="0 0.08 -0.02" rpy="0 0 0"/>',
            '    <axis xyz="0 0 1"/>',
            f'    <limit lower="{default_limit["lower"]:.6f}" upper="{default_limit["upper"]:.6f}" effort="{default_limit["effort"]:.1f}" velocity="{default_limit["velocity"]:.1f}"/>',
            '  </joint>',
            '  <link name="front_right_hip">',
            '    <inertial>',
            '      <origin xyz="0 0 0" rpy="0 0 0"/>',
            f'      <mass value="{hip_mass:.6f}"/>',
            f'      <inertia ixx="{hip_ixx:.8f}" iyy="{hip_iyy:.8f}" izz="{hip_izz:.8f}" ixy="0" ixz="0" iyz="0"/>',
            '    </inertial>',
            '  </link>',
            '  <joint name="front_right_hip_joint" type="revolute">',
            '    <parent link="body_0"/>',
            '    <child link="front_right_hip"/>',
            '    <origin xyz="0 -0.08 -0.02" rpy="0 0 0"/>',
            '    <axis xyz="0 0 1"/>',
            f'    <limit lower="{default_limit["lower"]:.6f}" upper="{default_limit["upper"]:.6f}" effort="{default_limit["effort"]:.1f}" velocity="{default_limit["velocity"]:.1f}"/>',
            '  </joint>'
        ]

    lines.append('</robot>')

    # write to file
    with open(path, 'w') as f:
        f.write("\n".join(lines))

    # Basic validation: attempt to load the URDF quickly into a DIRECT pybullet client
    if not validate_urdf_load(path):
        raise RuntimeError(f"URDF validation failed for file: {path}")

    return

def validate_urdf_load(urdf_path):
    """
    Attempt to load the URDF into a temporary DIRECT PyBullet client.
    Return True if load succeeds, False otherwise (and print diagnostics).
    """
    client = None
    try:
        client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
        # try to load; if it fails PyBullet will raise an error
        uid = p.loadURDF(urdf_path, physicsClientId=client)
        if uid < 0:
            print(f"[validate_urdf] loadURDF returned invalid uid {uid} for {urdf_path}")
            return False
        # success
        p.removeBody(uid, physicsClientId=client)
        p.disconnect(client)
        return True
    except Exception as e:
        print(f"[validate_urdf] Failed to load URDF '{urdf_path}': {e}")
        if client is not None:
            try:
                p.disconnect(client)
            except Exception:
                pass
        return False

# ---------------------------
# Spawn helper
# ---------------------------
LizardInstance = namedtuple('LizardInstance', ['body_uid', 'joint_indices', 'start_pos', 'link_name_map', 'head_link_index'])

def spawn_lizard_from_urdf(client, urdf_path, base_position, base_orientation=(0,0,0,1), global_scale=1.0):
    try:
        uid = p.loadURDF(urdf_path, basePosition=base_position, baseOrientation=base_orientation,
                         useFixedBase=False, globalScaling=global_scale, physicsClientId=client)
    except Exception as e:
        raise RuntimeError(f"Failed to spawn URDF '{urdf_path}': {e}")
    num_joints = p.getNumJoints(uid, physicsClientId=client)
    joint_indices = []
    link_name_map = {}
    head_link_index = None
    for ji in range(num_joints):
        info = p.getJointInfo(uid, ji, physicsClientId=client)
        jtype = info[2]
        link_name_raw = info[12]
        link_name = link_name_raw.decode() if isinstance(link_name_raw, (bytes, bytearray)) else str(link_name_raw)
        link_name_map[ji] = link_name
        if jtype == p.JOINT_REVOLUTE:
            joint_indices.append(ji)
        if 'head' in link_name.lower() or 'head_joint' in str(info[1]).lower():
            head_link_index = ji
    start_pos = p.getBasePositionAndOrientation(uid, physicsClientId=client)[0]
    return LizardInstance(body_uid=uid, joint_indices=joint_indices, start_pos=start_pos, link_name_map=link_name_map, head_link_index=head_link_index)

# ---------------------------
# Rollout logger
# ---------------------------
class RolloutLogger:
    def __init__(self, outdir, rollout_name, header_fields):
        self.outdir = outdir
        os.makedirs(outdir, exist_ok=True)
        self.csv_path = os.path.join(outdir, f"{rollout_name}.csv")
        self.frame_dir = os.path.join(outdir, f"{rollout_name}_frames")
        os.makedirs(self.frame_dir, exist_ok=True)
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(header_fields)
        self.step_count = 0

    def log_step(self, row):
        self.writer.writerow(row)
        self.step_count += 1
        if self.step_count % CSV_FLUSH_EVERY == 0:
            self.csv_file.flush()

    def save_frame(self, frame_arr, frame_index):
        img = Image.fromarray(frame_arr[:, :, :3])
        img_path = os.path.join(self.frame_dir, f"frame_{frame_index:06d}.png")
        img.save(img_path)

    def close(self):
        try:
            self.csv_file.close()
        except Exception:
            pass

# ---------------------------
# Evaluation
# ---------------------------
def evaluate_population(client, controllers, urdf_files, generation_idx,
                        debug_outdir, perturb_init_pose=0.0, sensor_noise=0.0, mass_perturb=0.0,
                        capture_frames=False, seed_override=None):
    if seed_override is not None:
        random.seed(seed_override)
        np.random.seed(seed_override)

    p.resetSimulation(physicsClientId=client)
    p.setGravity(0, 0, WORLD_GRAVITY, physicsClientId=client)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
    plane_id = p.loadURDF("plane.urdf", physicsClientId=client)

    WALL_THICKNESS = 0.2
    WALL_HEIGHT = 2.0
    ARENA_RADIUS = 1.5
    wall_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[WALL_THICKNESS/2, ARENA_RADIUS+0.5, WALL_HEIGHT/2], physicsClientId=client)
    left_wall = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=wall_col, basePosition=[-ARENA_RADIUS - WALL_THICKNESS/2, 0, WALL_HEIGHT/2], physicsClientId=client)
    right_wall = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=wall_col, basePosition=[ARENA_RADIUS + WALL_THICKNESS/2, 0, WALL_HEIGHT/2], physicsClientId=client)
    tall_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[ARENA_RADIUS+0.5, WALL_THICKNESS/2, WALL_HEIGHT/2], physicsClientId=client)
    front_wall = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=tall_col, basePosition=[0, ARENA_RADIUS + WALL_THICKNESS/2, WALL_HEIGHT/2], physicsClientId=client)
    back_wall = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=tall_col, basePosition=[0, -ARENA_RADIUS - WALL_THICKNESS/2, WALL_HEIGHT/2], physicsClientId=client)
    wall_ids = [left_wall, right_wall, front_wall, back_wall]

    agents = []
    agent_controllers = [c.copy() for c in controllers]

    center_x = 0.0
    center_y = 0.0
    n_agents = len(agent_controllers)
    radius = 0.5 + (0.5 * math.sqrt(n_agents))
    for i, ctl in enumerate(agent_controllers):
        angle = (2.0 * math.pi * i) / float(n_agents)
        px = center_x + radius * math.cos(angle)
        py = center_y + radius * math.sin(angle)
        delta_x = random.uniform(-perturb_init_pose, perturb_init_pose)
        delta_y = random.uniform(-perturb_init_pose, perturb_init_pose)
        delta_yaw = random.uniform(-perturb_init_pose, perturb_init_pose)
        pos = (px + delta_x, py + delta_y, 0.12)
        ori = p.getQuaternionFromEuler((0, 0, delta_yaw))
        urdf_file = urdf_files[i % len(urdf_files)]
        uid = spawn_lizard_from_urdf(client, urdf_file, base_position=pos, base_orientation=ori)
        agents.append(uid)
        p.resetBaseVelocity(uid.body_uid, linearVelocity=[random.uniform(-0.05,0.05), random.uniform(-0.05,0.05), 0.0], physicsClientId=client)

        if mass_perturb and mass_perturb > 0.0:
            scale_factor = 1.0 + random.uniform(-mass_perturb, mass_perturb)
            try:
                p.changeDynamics(uid.body_uid, -1, mass=scale_factor * 1.0, physicsClientId=client)
                for ji in range(p.getNumJoints(uid.body_uid, physicsClientId=client)):
                    p.changeDynamics(uid.body_uid, ji, mass=scale_factor * 0.1, physicsClientId=client)
            except Exception:
                pass

    start_positions = [p.getBasePositionAndOrientation(a.body_uid, physicsClientId=client)[0] for a in agents]
    yaw_at_last_head_contact = [None for _ in agents]
    accumulated_yaw_change_after_hit = [0.0 for _ in agents]
    head_contact_active = [False for _ in agents]
    contact_cooldown_steps = [0 for _ in agents]

    dt = SIMULATION_STEP
    t = 0.0

    joints_per_agent = [a.joint_indices for a in agents]

    logger_list = []
    for i, agent in enumerate(agents):
        rollout_name = f"gen{generation_idx}_ind{i}_rollout"
        lg = RolloutLogger(debug_outdir, rollout_name,
                           header_fields=[
                               'step', 'time', 'base_x','base_y','base_z',
                               'roll','pitch','yaw',
                               'linvx','linvy','linvz','angvx','angvy','angvz',
                               'head_contact_flag',
                               'joint_indices','joint_positions','joint_targets',
                               'contact_forces'])
        logger_list.append(lg)

    cam_distance = 1.0
    cam_yaw = 0
    cam_pitch = -30
    up_axis_index = 2

    for step in range(STEPS_PER_GEN):
        for idx, agent in enumerate(agents):
            ctl = agent_controllers[idx]
            base_pos, base_ori = p.getBasePositionAndOrientation(agent.body_uid, physicsClientId=client)
            base_lin_vel, base_ang_vel = p.getBaseVelocity(agent.body_uid, physicsClientId=client)
            roll, pitch, yaw = p.getEulerFromQuaternion(base_ori)
            head_contact = False
            if agent.head_link_index is not None:
                for wid in wall_ids:
                    contacts = p.getContactPoints(bodyA=agent.body_uid, bodyB=wid, linkIndexA=agent.head_link_index, physicsClientId=client)
                    if contacts:
                        head_contact = True
                        break
            hv = 1.0 if head_contact else 0.0
            sensors = np.array([roll, pitch, base_lin_vel[0], base_lin_vel[1], hv, math.sin(t), math.cos(t)])
            if sensor_noise and sensor_noise > 0.0:
                sensors = sensors + np.random.randn(*sensors.shape) * sensor_noise
            sensors = sensors / np.array([1.0, 1.0, 2.0, 2.0, 1.0, 1.0, 1.0])
            out = ctl.forward(sensors)

            jinds = joints_per_agent[idx] if idx < len(joints_per_agent) else []
            if out.size < len(jinds):
                out = np.tile(out, int(math.ceil(len(jinds) / out.size)))[:len(jinds)]
            elif out.size > len(jinds):
                out = out[:len(jinds)]

            joint_positions = []
            for jlocal, joint_index in enumerate(jinds):
                target = float(out[jlocal]) * 0.9
                try:
                    p.setJointMotorControl2(agent.body_uid, joint_index, controlMode=p.POSITION_CONTROL, targetPosition=target, force=60, physicsClientId=client)
                except Exception:
                    pass
                try:
                    js = p.getJointState(agent.body_uid, joint_index, physicsClientId=client)
                    joint_positions.append(js[0])
                except Exception:
                    joint_positions.append(0.0)

            if head_contact and contact_cooldown_steps[idx] == 0:
                yaw_at_last_head_contact[idx] = yaw
                head_contact_active[idx] = True
                contact_cooldown_steps[idx] = int(0.25 / dt)
            if head_contact_active[idx] and yaw_at_last_head_contact[idx] is not None:
                yaw_delta = yaw - yaw_at_last_head_contact[idx]
                while yaw_delta > math.pi: yaw_delta -= 2*math.pi
                while yaw_delta < -math.pi: yaw_delta += 2*math.pi
                accumulated_yaw_change_after_hit[idx] += abs(yaw_delta)
                yaw_at_last_head_contact[idx] = yaw
            if contact_cooldown_steps[idx] > 0:
                contact_cooldown_steps[idx] -= 1
            if not head_contact and contact_cooldown_steps[idx] == 0:
                head_contact_active[idx] = False
                yaw_at_last_head_contact[idx] = None

            contact_forces = []
            try:
                all_contacts = p.getContactPoints(bodyA=agent.body_uid, physicsClientId=client)
                for c in all_contacts:
                    contact_forces.append(float(c[9]))
            except Exception:
                contact_forces = []

            row = [
                step, t,
                base_pos[0], base_pos[1], base_pos[2],
                roll, pitch, yaw,
                base_lin_vel[0], base_lin_vel[1], base_lin_vel[2],
                base_ang_vel[0], base_ang_vel[1], base_ang_vel[2],
                1.0 if head_contact else 0.0,
                ";".join([str(j) for j in jinds]),
                ";".join([f"{jp:.4f}" for jp in joint_positions]),
                ";".join([f"{float(out[i]):.4f}" for i in range(len(jinds))]),
                ";".join([f"{cf:.4f}" for cf in contact_forces])
            ]
            logger_list[idx].log_step(row)

            if capture_frames and (step % FRAME_SAVE_EVERY == 0):
                view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=base_pos,
                                                                  distance=cam_distance,
                                                                  yaw=cam_yaw,
                                                                  pitch=cam_pitch,
                                                                  roll=0,
                                                                  upAxisIndex=up_axis_index,
                                                                  physicsClientId=client)
                proj_matrix = p.computeProjectionMatrixFOV(fov=60.0, aspect=float(MAX_FRAME_WIDTH)/float(MAX_FRAME_HEIGHT),
                                                           nearVal=0.01, farVal=10.0, physicsClientId=client)
                img = p.getCameraImage(width=MAX_FRAME_WIDTH, height=MAX_FRAME_HEIGHT, viewMatrix=view_matrix, projectionMatrix=proj_matrix, physicsClientId=client)
                w = img[0]; h = img[1]
                if img[2] is not None:
                    arr = np.reshape(np.array(img[2], dtype=np.uint8), (h, w, 4))
                    logger_list[idx].save_frame(arr, step)

        p.stepSimulation(physicsClientId=client)
        t += dt

    fitnesses = []
    end_positions = []
    upright_scores = []
    travel_scores = []
    turn_scores = []
    for i, agent in enumerate(agents):
        try:
            end_pos, end_ori = p.getBasePositionAndOrientation(agent.body_uid, physicsClientId=client)
            ex, ey, ez = end_pos
            sx, sy, sz = start_positions[i]
            dx = ex - sx
            dy = ey - sy
            travel_dist = math.sqrt(dx*dx + dy*dy)
            roll, pitch, yaw = p.getEulerFromQuaternion(end_ori)
            tilt = min(1.57, abs(roll) + abs(pitch))
            upright = max(0.0, 1.0 - (tilt / 1.57))
            turn_val = min(accumulated_yaw_change_after_hit[i], math.pi * 4) / (math.pi * 4)
        except Exception:
            travel_dist = 0.0
            upright = 0.0
            turn_val = 0.0
            end_pos = start_positions[i]
        fit = W_UPRIGHT * upright + W_TRAVEL * travel_dist + W_TURN_AFTER_HIT * turn_val
        fitnesses.append(fit)
        end_positions.append(end_pos)
        upright_scores.append(upright)
        travel_scores.append(travel_dist)
        turn_scores.append(turn_val)

    for lg in logger_list:
        lg.close()

    return fitnesses, agent_controllers, start_positions, end_positions, upright_scores, travel_scores, turn_scores

# ---------------------------
# Unit-test harness
# ---------------------------
def unit_test_controller(controller_path, urdf_morphology='simple', trials=5, seed=1234, capture_frames=False):
    if not os.path.exists(controller_path):
        print(f"[unit_test] controller file not found: {controller_path}")
        return

    with open(controller_path, 'rb') as f:
        weights = pickle.load(f)
    W1 = np.array(weights['W1'])
    b1 = np.array(weights['b1'])
    W2 = np.array(weights['W2'])
    b2 = np.array(weights['b2'])
    input_size = W1.shape[1]
    hidden_size = W1.shape[0]
    output_size = W2.shape[0]
    ctrl = MLPController(input_size, hidden_size, output_size, weights={'W1':W1, 'b1':b1, 'W2':W2, 'b2':b2})

    controllers = [ctrl]
    urdf_folder = os.path.join(OUTPUT_ROOT, "unit_test_urdfs")
    os.makedirs(urdf_folder, exist_ok=True)
    urdf_file = os.path.join(urdf_folder, "unit_test_lizard.urdf")
    write_lizard_urdf(urdf_file, morphology=urdf_morphology, name_prefix="unit_test", scale=1.0)

    client = p.connect(p.DIRECT)
    p.setTimeStep(SIMULATION_STEP, physicsClientId=client)
    p.setGravity(0, 0, WORLD_GRAVITY, physicsClientId=client)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)

    debug_outdir = os.path.join(OUTPUT_ROOT, "unit_test_results")
    os.makedirs(debug_outdir, exist_ok=True)

    for tr in range(trials):
        trial_seed = seed + tr
        print(f"[unit_test] trial {tr+1}/{trials} seed={trial_seed}")
        fitnesses, used_controllers, starts, ends, uprights, travels, turns = evaluate_population(
            client, controllers, [urdf_file], generation_idx=f"unit_{tr+1}",
            debug_outdir=debug_outdir,
            perturb_init_pose=0.0, sensor_noise=0.0, mass_perturb=0.0,
            capture_frames=capture_frames, seed_override=trial_seed
        )
        print(f"  fitnesses={fitnesses} upright={uprights} travel={travels} turn={turns}")

    try:
        p.disconnect(client)
    except Exception:
        pass
    print("[unit_test] complete")

# ---------------------------
# Evolution CLI
# ---------------------------
def run_evolution_cli(args):
    input_size = 7
    output_size = 16
    hidden_size = HIDDEN_NEURONS

    set_global_seed(args.seed)

    controllers = [MLPController(input_size, hidden_size, output_size) for _ in range(POPULATION)]

    urdf_folder = os.path.join(OUTPUT_ROOT, "evo_urdfs")
    os.makedirs(urdf_folder, exist_ok=True)
    urdf_files = []
    for idx, ctl in enumerate(controllers):
        urdf_path = os.path.join(urdf_folder, f"init_lizard_{idx}.urdf")
        write_lizard_urdf(urdf_path, morphology=args.morphology, scale=1.0, name_prefix=f"init_{idx}", body_color=ctl.color)
        urdf_files.append(urdf_path)

    STATS_CSV = os.path.join(OUTPUT_ROOT, "generation_stats.csv")
    with open(STATS_CSV, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['generation', 'best_fitness', 'mean_fitness', 'median_fitness', 'best_upright', 'best_travel', 'best_turn'])

    BEST_MODEL_FILE = os.path.join(OUTPUT_ROOT, "best_model.pkl")
    client = p.connect(p.DIRECT)
    p.setTimeStep(SIMULATION_STEP, physicsClientId=client)
    p.setGravity(0, 0, WORLD_GRAVITY, physicsClientId=client)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)

    best_overall = None
    best_overall_fitness = -1.0
    best_overall_controller = None

    for gen in range(1, args.generations + 1):
        print(f"\n=== Generation {gen}/{args.generations} ===")
        fitnesses, used_controllers, starts, ends, uprights, travels, turns = evaluate_population(
            client, controllers, urdf_files, generation_idx=gen,
            debug_outdir=os.path.join(OUTPUT_ROOT, f"gen{gen}_logs"),
            perturb_init_pose=args.perturb_init_pose,
            sensor_noise=args.sensor_noise,
            mass_perturb=args.mass_perturb,
            capture_frames=args.capture_frames,
            seed_override=(args.seed + gen) if args.seed is not None else None
        )

        best_idx = int(np.argmax(fitnesses))
        best_fitness = float(fitnesses[best_idx])
        mean_fitness = float(np.mean(fitnesses))
        median_fitness = float(np.median(fitnesses))
        print(f"  best_fit={best_fitness:.4f} (idx {best_idx}) mean={mean_fitness:.4f}")

        with open(STATS_CSV, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([gen, best_fitness, mean_fitness, median_fitness, uprights[best_idx], travels[best_idx], turns[best_idx]])

        if best_fitness > best_overall_fitness:
            best_overall_fitness = best_fitness
            best_overall = {'generation': gen, 'index': best_idx, 'fitness': best_fitness}
            best_overall_controller = used_controllers[best_idx].copy()
            with open(BEST_MODEL_FILE, 'wb') as f:
                pickle.dump(best_overall_controller.weights, f)
            best_urdf = os.path.join(OUTPUT_ROOT, f"best_lizard_gen{gen}_ind{best_idx}.urdf")
            write_lizard_urdf(best_urdf, morphology=args.morphology, scale=1.0, name_prefix=f"best_gen{gen}_ind{best_idx}", body_color=used_controllers[best_idx].color)

        sorted_idx = np.argsort(fitnesses)[::-1]
        parents = [used_controllers[int(sorted_idx[i])] for i in range(max(1, ELITE_COUNT))]

        next_population = []
        for pidx in range(ELITE_COUNT):
            next_population.append(parents[pidx].copy())
        while len(next_population) < POPULATION:
            a = random.choice(parents)
            b = random.choice(parents)
            child = MLPController.crossover(a, b)
            child.mutate(rate=MUTATION_RATE, prob=MUTATION_PROB)
            child.color = getattr(a, 'color', getattr(b, 'color', (0.3,0.6,0.3,1.0)))
            next_population.append(child)

        controllers = next_population
        urdf_files = []
        for idx, ctl in enumerate(controllers):
            urdf_path = os.path.join(urdf_folder, f"gen{gen}_lizard_{idx}.urdf")
            write_lizard_urdf(urdf_path, morphology=args.morphology, scale=1.0, name_prefix=f"g{gen}_{idx}", body_color=ctl.color)
            urdf_files.append(urdf_path)

    try:
        p.disconnect(client)
    except Exception:
        pass

    print(f"\nBest overall: {best_overall}, saved model -> {BEST_MODEL_FILE}")
    return best_overall, best_overall_controller

# ---------------------------
# CLI parse
# ---------------------------
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--generations', type=int, default=GENERATIONS)
    parser.add_argument('--seed', type=int, default=SEED)
    parser.add_argument('--perturb_init_pose', type=float, default=0.0)
    parser.add_argument('--sensor_noise', type=float, default=0.0)
    parser.add_argument('--mass_perturb', type=float, default=0.0)
    parser.add_argument('--capture_frames', action='store_true')
    parser.add_argument('--morphology', choices=['default', 'simple'], default='simple')
    parser.add_argument('--unit_test', action='store_true')
    parser.add_argument('--controller', type=str, default='best_model.pkl')
    parser.add_argument('--trials', type=int, default=5)
    parser.add_argument('--capture_frames_unit', action='store_true')
    return parser.parse_args()

if __name__ == "__main__":
    args = parse_args()
    if args.unit_test:
        unit_test_controller(args.controller, urdf_morphology=args.morphology, trials=args.trials, seed=args.seed, capture_frames=args.capture_frames_unit)
    else:
        run_evolution_cli(args)
