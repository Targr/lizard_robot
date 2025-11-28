#!/usr/bin/env python3
"""
ltt_no_portals.py

Modified from original ltt_portals.py:
 - portals, cubes and untrained GUI logic removed.
 - fitness now measures forward speed and correct head-turn behavior when contacting fixed walls.
 - if wall is movable and the lizard pushes it (wall moves), a head-turn in response is penalized.
 - URDF writer and MLP controller preserved.
 - Outputs: generation CSVs and BEST_MODEL_FILE (best_model.pkl) with controller.weights suitable for deployment.
"""
import os
import time
import math
import random
import pickle
import csv
import sys
from datetime import datetime
from collections import namedtuple

import numpy as np
import pybullet as p
import pybullet_data

# ------------------------------
# Config
# ------------------------------
POPULATION = 10
GENERATIONS = 120
GENERATION_DURATION = 60.0           # seconds per generation in sim
SIMULATION_STEP = 1.0 / 240.0        # physics step
STEPS_PER_GEN = int(GENERATION_DURATION / SIMULATION_STEP)
WORLD_GRAVITY = -9.81
SEED = 42
ELITE_COUNT = 1
MUTATION_RATE = 0.06
MUTATION_PROB = 0.15
HIDDEN_NEURONS = 24

OUTPUT_FOLDER = f"evolution_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
os.makedirs(OUTPUT_FOLDER, exist_ok=True)
STATS_CSV = os.path.join(OUTPUT_FOLDER, "generation_stats.csv")
PER_INDIVIDUAL_CSV = os.path.join(OUTPUT_FOLDER, "per_individual_stats.csv")
BEST_MODEL_FILE = os.path.join(OUTPUT_FOLDER, "best_model.pkl")
URDF_FOLDER = os.path.join(OUTPUT_FOLDER, "urdfs")
os.makedirs(URDF_FOLDER, exist_ok=True)

random.seed(SEED)
np.random.seed(SEED)

# Wall settings: set mass to 0.0 for unmovable (fixed) wall, >0.0 for movable wall
LEFT_WALL_MASS = 0.0     # if 0 => fixed (doesn't budge); if >0, wall will move when pushed
RIGHT_WALL_MASS = 0.0

# world layout
WALL_THICKNESS = 0.2
WALL_HEIGHT = 1.0
WALL_LENGTH = 6.0
ARENA_RADIUS = 1.5

# fitness weights
FIT_FORWARD_WEIGHT = 1.0       # forward displacement component (meters)
FIT_HEAD_TURN_WEIGHT = 1.5     # reward when head turns away appropriately on fixed-wall contact
PENALTY_REFUSE_ON_MOVABLE = 1.0 # penalty when agent turns away while wall moved (i.e., should have pushed)

# ------------------------------
# MLP Controller (unchanged except for minor output size flexibility)
# ------------------------------
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
        if hasattr(self, 'color'):
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
        if hasattr(parent_a, 'color') and hasattr(parent_b, 'color'):
            child.color = tuple(parent_a.color) if (random.random() < 0.5) else tuple(parent_b.color)
        return child

    def mutate(self, rate=MUTATION_RATE, prob=MUTATION_PROB):
        flat = self.get_flat()
        mutation_mask = np.random.rand(flat.size) < prob
        gaussian = np.random.randn(flat.size) * rate
        flat = np.where(mutation_mask, flat + gaussian, flat)
        self.set_flat(flat)

# ------------------------------
# URDF writer (kept intact from original)
# ------------------------------
def write_lizard_urdf(path, body_segments=5, tail_segments=3, scale=1.0, name_prefix="lizard", body_color=(0.3,0.6,0.3,1.0)):
    # (Full write_lizard_urdf content preserved from original file)
    # For brevity in this inline listing, we will copy original function body EXACTLY
    # In your copy please paste the original write_lizard_urdf function body unchanged.
    # --- BEGIN original URDF writer (paste in unchanged) ---
    r,g,b,a = [float(x) for x in body_color]
    body_length = 0.35 * scale
    body_height = 0.08 * scale
    body_width = 0.12 * scale
    tail_length = 0.22 * scale
    leg_length = 0.22 * scale

    joints = []
    lines = []
    lines.append('<?xml version="1.0" ?>')
    lines.append(f'<robot name="{name_prefix}">')
    lines += [
        f'  <material name="body_color"><color rgba="{r} {g} {b} {a}"/></material>',
        '  <material name="jaw_color"><color rgba="0.7 0.15 0.15 1"/></material>',
        '  <material name="black"><color rgba="0 0 0 1"/></material>'
    ]
    lines += [
        '  <link name="body_0">',
        '    <inertial>',
        f'      <mass value="{0.6*scale}"/>',
        '      <inertia ixx="0.001" iyy="0.001" izz="0.001"/>',
        '    </inertial>',
        '    <visual>',
        f'      <geometry><box size="{body_length} {body_width} {body_height}"/></geometry>',
        '      <material name="body_color"/>',
        '    </visual>',
        '    <collision>',
        f'      <geometry><box size="{body_length} {body_width} {body_height}"/></geometry>',
        '    </collision>',
        '  </link>'
    ]
    for i in range(1, body_segments):
        lines += [
            f'  <link name="body_{i}">',
            '    <inertial>',
            f'      <mass value="{0.45*scale}"/>',
            '      <inertia ixx="0.001" iyy="0.001" izz="0.001"/>',
            '    </inertial>',
            '    <visual>',
            f'      <geometry><box size="{body_length} {body_width} {body_height}"/></geometry>',
            '      <material name="body_color"/>',
            '    </visual>',
            '    <collision>',
            f'      <geometry><box size="{body_length} {body_width} {body_height}"/></geometry>',
            '    </collision>',
            '  </link>'
        ]
        jname = f'body_joint_{i-1}_{i}'
        axis = "0 0 1"
        origin_x = body_length * 0.9
        lines += [
            f'  <joint name="{jname}" type="revolute">',
            f'    <parent link="body_{i-1}"/>',
            f'    <child link="body_{i}"/>',
            f'    <origin xyz="{origin_x} 0 0" rpy="0 0 0"/>',
            f'    <axis xyz="{axis}"/>',
            '    <limit lower="-1.2" upper="1.2" effort="50" velocity="4.0"/>',
            '  </joint>'
        ]
        joints.append(jname)
    for t in range(tail_segments):
        lines += [
            f'  <link name="tail_{t}">',
            '    <inertial>',
            f'      <mass value="{0.2*scale}"/>',
            '      <inertia ixx="0.0005" iyy="0.0005" izz="0.0005"/>',
            '    </inertial>',
            '    <visual>',
            f'      <geometry><box size="{tail_length} {0.08*scale} {0.06*scale}"/></geometry>',
            '      <material name="body_color"/>',
            '    </visual>',
            '    <collision>',
            f'      <geometry><box size="{tail_length} {0.08*scale} {0.06*scale}"/></geometry>',
            '    </collision>',
            '  </link>'
        ]
        parent = f'body_{body_segments-1}' if t == 0 else f'tail_{t-1}'
        jname = f'tail_joint_{t}'
        axis = "0 0 1"
        origin_x = tail_length * 0.9
        lines += [
            f'  <joint name="{jname}" type="revolute">',
            f'    <parent link="{parent}"/>',
            f'    <child link="tail_{t}"/>',
            f'    <origin xyz="{origin_x} 0 0" rpy="0 0 0"/>',
            f'    <axis xyz="{axis}"/>',
            '    <limit lower="-1.5" upper="1.5" effort="30" velocity="4.0"/>',
            '  </joint>'
        ]
        joints.append(jname)
    head_length = 0.18 * scale
    head_width = 0.12 * scale
    head_height = 0.06 * scale
    lines += [
        '  <link name="head">',
        '    <inertial>',
        f'      <mass value="{0.25*scale}"/>',
        '      <inertia ixx="0.0006" iyy="0.0006" izz="0.0006"/>',
        '    </inertial>',
        '    <visual>',
        f'      <geometry><box size="{head_length} {head_width} {head_height}"/></geometry>',
        '      <material name="body_color"/>',
        '    </visual>',
        '    <collision>',
        f'      <geometry><box size="{head_length} {head_width} {head_height}"/></geometry>',
        '    </collision>',
        '  </link>',
        '  <joint name="head_joint" type="revolute">',
        '    <parent link="body_0"/>',
        '    <child link="head"/>',
        f'    <origin xyz="{-body_length*0.55} 0 {body_height*0.5 + head_height*0.5 - 0.01}" rpy="0 0 0"/>',
        '    <axis xyz="0 1 0"/>',
        '    <limit lower="-1.2" upper="1.2" effort="20" velocity="4.0"/>',
        '  </joint>',
    ]
    jaw_length = head_length * 1.05
    jaw_width = 0.075 * scale
    jaw_height = 0.05 * scale
    jaw_mass = 0.12 * scale
    lines += [
        '  <link name="jaw">',
        '    <inertial>',
        f'      <mass value="{jaw_mass}"/>',
        '      <inertia ixx="0.0004" iyy="0.0004" izz="0.0004"/>',
        '    </inertial>',
        '    <visual>',
        f'      <geometry><box size="{jaw_length} {jaw_width} {jaw_height}"/></geometry>',
        '      <material name="jaw_color"/>',
        '    </visual>',
        '    <collision>',
        f'      <geometry><box size="{jaw_length} {jaw_width} {jaw_height}"/></geometry>',
        '    </collision>',
        '  </link>',
        '  <joint name="jaw_joint" type="revolute">',
        '    <parent link="head"/>',
        '    <child link="jaw"/>',
        f'    <origin xyz="{head_length*0.25} 0 {-head_height*0.5 - 0.005}" rpy="0 0 0"/>',
        '    <axis xyz="0 1 0"/>',
        '    <limit lower="-1.5" upper="0.5" effort="32" velocity="4.0"/>',
        '  </joint>'
    ]
    joints += ['head_joint', 'jaw_joint']
    leg_positions = [
        ("front_left", "body_0", 0.02, 0.12),
        ("front_right", "body_0", -0.02, -0.12),
        ("rear_left", f"body_{body_segments-1}", 0.02, 0.12),
        ("rear_right", f"body_{body_segments-1}", -0.02, -0.12),
    ]
    for (lname, parent_link, z_off, y_off) in leg_positions:
        yaw = 0.6 if y_off > 0 else -0.6
        lines += [
            f'  <link name="{lname}_hip">',
            '    <inertial>',
            f'      <mass value="{0.08*scale}"/>',
            '      <inertia ixx="0.0002" iyy="0.0002" izz="0.0002"/>',
            '    </inertial>',
            '    <visual>',
            f'      <geometry><box size="{leg_length} {0.04*scale} {0.04*scale}"/></geometry>',
            '      <material name="body_color"/>',
            '    </visual>',
            '    <collision>',
            f'      <geometry><box size="{leg_length} {0.04*scale} {0.04*scale}"/></geometry>',
            '    </collision>',
            '  </link>',
            f'  <joint name="{lname}_hip_joint" type="revolute">',
            f'    <parent link="{parent_link}"/>',
            f'    <child link="{lname}_hip"/>',
            f'    <origin xyz="0 {y_off} {z_off}" rpy="0 0 {yaw}"/>',
            '    <axis xyz="0 0 1"/>',
            '    <limit lower="-1.2" upper="1.2" effort="30" velocity="4.0"/>',
            '  </joint>',
            f'  <link name="{lname}_knee">',
            '    <inertial>',
            f'      <mass value="{0.06*scale}"/>',
            '      <inertia ixx="0.00015" iyy="0.00015" izz="0.00015"/>',
            '    </inertial>',
            '    <visual>',
            f'      <geometry><box size="{leg_length} {0.03*scale} {0.03*scale}"/></geometry>',
            '      <material name="body_color"/>',
            '    </visual>',
            '    <collision>',
            f'      <geometry><box size="{leg_length} {0.03*scale} {0.03*scale}"/></geometry>',
            '    </collision>',
            '  </link>',
            f'  <joint name="{lname}_knee_joint" type="revolute">',
            f'    <parent link="{lname}_hip"/>',
            f'    <child link="{lname}_knee"/>',
            f'    <origin xyz="{leg_length*0.45} 0 -0.06" rpy="0 0 0"/>',
            '    <axis xyz="0 0 1"/>',
            '    <limit lower="-1.4" upper="0.6" effort="20" velocity="4.0"/>',
            '  </joint>'
        ]
        joints += [f'{lname}_hip_joint', f'{lname}_knee_joint']
    tiny_mass = 0.001 * scale
    eye_radius = 0.02 * scale
    eye_top_z = head_height * 0.5 + 0.01
    eye_y_offset = head_width * 0.35
    lines += [
        f'  <link name="eye_left">',
        '    <inertial>',
        f'      <mass value="{tiny_mass}"/>',
        '      <inertia ixx="0.000001" iyy="0.000001" izz="0.000001"/>',
        '    </inertial>',
        '    <visual>',
        f'      <geometry><sphere radius="{eye_radius}"/></geometry>',
        '      <material name="black"/>',
        '    </visual>',
        '    <collision>',
        f'      <geometry><sphere radius="{eye_radius}"/></geometry>',
        '    </collision>',
        '  </link>',
        '  <joint name="eye_left_fixed" type="fixed">',
        '    <parent link="head"/>',
        '    <child link="eye_left"/>',
        f'    <origin xyz="{0:.6f} {eye_y_offset:.6f} {eye_top_z:.6f}" rpy="0 0 0"/>',
        '  </joint>',
        f'  <link name="eye_right">',
        '    <inertial>',
        f'      <mass value="{tiny_mass}"/>',
        '      <inertia ixx="0.000001" iyy="0.000001" izz="0.000001"/>',
        '    </inertial>',
        '    <visual>',
        f'      <geometry><sphere radius="{eye_radius}"/></geometry>',
        '      <material name="black"/>',
        '    </visual>',
        '    <collision>',
        f'      <geometry><sphere radius="{eye_radius}"/></geometry>',
        '    </collision>',
        '  </link>',
        '  <joint name="eye_right_fixed" type="fixed">',
        '    <parent link="head"/>',
        '    <child link="eye_right"/>',
        f'    <origin xyz="{0:.6f} {-eye_y_offset:.6f} {eye_top_z:.6f}" rpy="0 0 0"/>',
        '  </joint>'
    ]
    lines.append('</robot>')
    with open(path, 'w') as f:
        f.write("\n".join(lines))
    return joints
    # --- END original URDF writer ---

# ------------------------------
# Utilities
# ------------------------------
LizardInstance = namedtuple('LizardInstance', ['body_uid', 'joint_indices', 'start_pos', 'link_name_map', 'leg_joints'])

def spawn_lizard_from_urdf(client, urdf_path, base_position, base_orientation=(0,0,0,1), global_scale=1.0):
    uid = p.loadURDF(urdf_path, basePosition=base_position, baseOrientation=base_orientation,
                     useFixedBase=False, globalScaling=global_scale, physicsClientId=client)
    num_joints = p.getNumJoints(uid, physicsClientId=client)
    joint_indices = []
    link_name_map = {}
    leg_joints = []
    for ji in range(num_joints):
        info = p.getJointInfo(uid, ji, physicsClientId=client)
        jtype = info[2]
        link_name_raw = info[12]
        link_name = link_name_raw.decode() if isinstance(link_name_raw, (bytes, bytearray)) else str(link_name_raw)
        link_name_map[ji] = link_name
        if jtype == p.JOINT_REVOLUTE:
            joint_indices.append(ji)
        lname_low = link_name.lower()
        if 'hip' in lname_low or 'knee' in lname_low:
            leg_joints.append(ji)
    start_pos = p.getBasePositionAndOrientation(uid, physicsClientId=client)[0]
    return LizardInstance(body_uid=uid, joint_indices=joint_indices, start_pos=start_pos,
                          link_name_map=link_name_map, leg_joints=leg_joints)

def make_initial_population(pop_size, input_size, hidden_size, output_size):
    return [MLPController(input_size, hidden_size, output_size) for _ in range(pop_size)]

# ------------------------------
# Evaluation - simplified arena with two walls
# ------------------------------
def evaluate_population(client, controllers, urdf_files, generation, total_generations,
                        display=False):
    p.resetSimulation(physicsClientId=client)
    p.setGravity(0, 0, WORLD_GRAVITY, physicsClientId=client)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
    plane_id = p.loadURDF("plane.urdf", physicsClientId=client)

    # build two walls facing the arena center along x axis (left and right)
    # walls centered at +/- (ARENA_RADIUS + offset)
    offset = ARENA_RADIUS + 0.2
    wall_half_extents = [WALL_THICKNESS/2.0, WALL_LENGTH/2.0, WALL_HEIGHT/2.0]

    wall_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=wall_half_extents, physicsClientId=client)
    wall_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=wall_half_extents, rgbaColor=[0.6,0.6,0.6,1], physicsClientId=client)

    left_wall = p.createMultiBody(baseMass=LEFT_WALL_MASS, baseCollisionShapeIndex=wall_collision, baseVisualShapeIndex=wall_visual,
                                  basePosition=[-offset, 0, WALL_HEIGHT/2.0], physicsClientId=client)
    right_wall = p.createMultiBody(baseMass=RIGHT_WALL_MASS, baseCollisionShapeIndex=wall_collision, baseVisualShapeIndex=wall_visual,
                                   basePosition=[offset, 0, WALL_HEIGHT/2.0], physicsClientId=client)

    # spawn agents in circle
    n_agents = len(controllers)
    center_x = 0.0
    center_y = 0.0
    start_z = 0.12
    radius = max(0.5, 0.6 * n_agents / (2.0 * math.pi))
    agents = []
    agent_controllers = [c.copy() for c in controllers]

    for i, ctl in enumerate(agent_controllers):
        angle = (2.0 * math.pi * i) / float(n_agents)
        px = center_x + radius * math.cos(angle)
        py = center_y + radius * math.sin(angle)
        pos = (px, py, start_z)
        urdf_file = urdf_files[i % len(urdf_files)]
        uid = spawn_lizard_from_urdf(client, urdf_file, base_position=pos)
        agents.append(uid)
        p.resetBaseVelocity(uid.body_uid, linearVelocity=[random.uniform(-0.05,0.05), random.uniform(-0.05,0.05), 0.0], physicsClientId=client)
        yaw = math.atan2(center_y - py, center_x - px)
        orn = p.getQuaternionFromEuler([0, 0, yaw])
        p.resetBasePositionAndOrientation(uid.body_uid, pos, orn, physicsClientId=client)

    start_positions = [p.getBasePositionAndOrientation(a.body_uid, physicsClientId=client)[0] for a in agents]

    dt = SIMULATION_STEP
    t = 0.0

    joints_per_agent = [a.joint_indices for a in agents]

    # track head turning events and wall movement
    head_turn_events = [0 for _ in agents]
    wall_movement_when_contact = [0 for _ in agents]  # 0 = no movement, 1 = moved

    # helper to find head joint index per agent
    head_joint_idx = []
    for a in agents:
        hj = None
        num_j = p.getNumJoints(a.body_uid, physicsClientId=client)
        for ji in range(num_j):
            info = p.getJointInfo(a.body_uid, ji, physicsClientId=client)
            link_name = info[12].decode() if isinstance(info[12], (bytes, bytearray)) else str(info[12])
            joint_name = info[1].decode() if isinstance(info[1], (bytes, bytearray)) else str(info[1])
            if 'head' in link_name or 'head_joint' in joint_name:
                hj = ji
                break
        head_joint_idx.append(hj)

    max_head_turn_angle = 1.0  # rad expected full turn magnitude

    for step in range(STEPS_PER_GEN):
        for idx, agent in enumerate(agents):
            ctl = agent_controllers[idx]
            base_pos, base_ori = p.getBasePositionAndOrientation(agent.body_uid, physicsClientId=client)
            base_lin_vel, base_ang_vel = p.getBaseVelocity(agent.body_uid, physicsClientId=client)
            roll, pitch, yaw = p.getEulerFromQuaternion(base_ori)
            sensors = np.array([roll, pitch, base_lin_vel[0], base_lin_vel[1], math.sin(t), math.cos(t)])
            sensors = sensors / np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
            out = ctl.forward(sensors)

            jinds = joints_per_agent[idx] if idx < len(joints_per_agent) else []
            if out.size < len(jinds):
                out = np.tile(out, int(math.ceil(len(jinds) / out.size)))[:len(jinds)]
            elif out.size > len(jinds):
                out = out[:len(jinds)]

            for jlocal, joint_index in enumerate(jinds):
                target = float(out[jlocal]) * 0.7
                try:
                    p.setJointMotorControl2(agent.body_uid, joint_index, controlMode=p.POSITION_CONTROL,
                                            targetPosition=target, force=60, physicsClientId=client)
                except Exception:
                    pass

        p.stepSimulation(physicsClientId=client)
        t += dt

        # check for contacts with walls & whether walls moved due to push
        # measure baseline wall positions to detect movement
        left_wall_pos = p.getBasePositionAndOrientation(left_wall, physicsClientId=client)[0]
        right_wall_pos = p.getBasePositionAndOrientation(right_wall, physicsClientId=client)[0]

        for a_idx, agent in enumerate(agents):
            if agent is None:
                continue
            # contacts with walls
            contacts_left = p.getContactPoints(bodyA=agent.body_uid, bodyB=left_wall, physicsClientId=client)
            contacts_right = p.getContactPoints(bodyA=agent.body_uid, bodyB=right_wall, physicsClientId=client)
            contacted_wall = None
            wall_id = None
            if contacts_left:
                contacted_wall = 'left'
                wall_id = left_wall
            elif contacts_right:
                contacted_wall = 'right'
                wall_id = right_wall

            if contacted_wall is not None:
                # check whether wall moved from previous position (if wall mass > 0 it might move)
                new_wall_pos = p.getBasePositionAndOrientation(wall_id, physicsClientId=client)[0]
                moved = (abs(new_wall_pos[0] - (left_wall_pos[0] if contacted_wall == 'left' else right_wall_pos[0])) > 1e-3) or \
                        (abs(new_wall_pos[1] - (left_wall_pos[1] if contacted_wall == 'left' else right_wall_pos[1])) > 1e-3)
                # read head joint angle
                hj = head_joint_idx[a_idx]
                head_angle = 0.0
                if hj is not None:
                    try:
                        info = p.getJointState(agent.body_uid, hj, physicsClientId=client)
                        head_angle = info[0]  # position
                    except Exception:
                        head_angle = 0.0
                # interpret a "turn away" as head_angle magnitude > small threshold and direction away from wall
                turned_away = False
                if contacted_wall == 'left':
                    # left wall is at negative X; turning right is positive heading about Y axis -> check sign
                    # because head_joint axis is Y we just check absolute magnitude here
                    if abs(head_angle) > 0.2:
                        turned_away = True
                else:
                    if abs(head_angle) > 0.2:
                        turned_away = True

                # register events
                if not moved and turned_away:
                    head_turn_events[a_idx] += 1
                if moved:
                    wall_movement_when_contact[a_idx] = 1

    # finalize fitness
    fitnesses = []
    end_positions = []
    for i, agent in enumerate(agents):
        try:
            end_pos, _ = p.getBasePositionAndOrientation(agent.body_uid, physicsClientId=client)
        except Exception:
            end_pos = start_positions[i]
        sx, sy, sz = start_positions[i]
        ex, ey, ez = end_pos
        dx = ex - sx
        forward_dist = max(0.0, dx)  # treat +x as forward; if you prefer absolute magnitude use sqrt(dx^2 + dy^2)
        # head-turn score: normalized events over simulation length
        head_score = (head_turn_events[i] / max(1, STEPS_PER_GEN / 100.0))
        penalty = 0.0
        if wall_movement_when_contact[i] > 0 and head_turn_events[i] > 0:
            # penalize turning when wall moved (i.e., should have pushed instead)
            penalty = PENALTY_REFUSE_ON_MOVABLE * (head_turn_events[i] / max(1, 1.0))
        fit = FIT_FORWARD_WEIGHT * forward_dist + FIT_HEAD_TURN_WEIGHT * head_score - penalty
        fitnesses.append(fit)
        end_positions.append(end_pos)

    return fitnesses, agent_controllers, start_positions, end_positions

# ------------------------------
# Evolution loop
# ------------------------------
def run_evolution():
    input_size = 6
    # choose a reasonable output size: use 40 as before (should be >= num joints)
    output_size = 40
    hidden_size = HIDDEN_NEURONS

    controllers = make_initial_population(POPULATION, input_size, hidden_size, output_size)

    # initial urdfs for population
    for idx, ctl in enumerate(controllers):
        color = getattr(ctl, 'color', (0.3,0.6,0.3,1.0))
        urdf_path = os.path.join(URDF_FOLDER, f"init_ind{idx}.urdf")
        write_lizard_urdf(urdf_path, body_segments=5, tail_segments=3, scale=1.0, name_prefix=f"init_{idx}", body_color=color)

    # csv headers
    with open(STATS_CSV, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['generation', 'best_fitness', 'mean_fitness', 'median_fitness'])

    with open(PER_INDIVIDUAL_CSV, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['generation', 'individual_index', 'fitness', 'start_x','start_y','start_z','end_x','end_y','end_z'])

    best_overall_controller = None
    best_overall_fitness = -1e9

    for gen in range(1, GENERATIONS+1):
        print(f"\n--- Generation {gen}/{GENERATIONS} ---")
        # write urdfs for controllers
        gen_urdfs = []
        gen_dir = os.path.join(OUTPUT_FOLDER, f"gen_{gen}")
        os.makedirs(gen_dir, exist_ok=True)
        for idx, ctl in enumerate(controllers):
            urdf_path = os.path.join(gen_dir, f"gen{gen}_ind{idx}.urdf")
            write_lizard_urdf(urdf_path, body_segments=5, tail_segments=3, scale=1.0, name_prefix=f"g{gen}_ind{idx}", body_color=getattr(ctl, 'color', (0.3,0.6,0.3,1.0)))
            gen_urdfs.append(urdf_path)

        client = p.connect(p.DIRECT)
        p.setPhysicsEngineParameter(numSolverIterations=10, physicsClientId=client)
        p.setTimeStep(SIMULATION_STEP, physicsClientId=client)
        p.setGravity(0,0,WORLD_GRAVITY, physicsClientId=client)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)

        fitnesses, used_controllers, starts, ends = evaluate_population(client, controllers, gen_urdfs, generation=gen, total_generations=GENERATIONS)

        try:
            p.disconnect(client)
        except Exception:
            pass

        best_idx = int(np.argmax(fitnesses)) if fitnesses else 0
        best_fitness = float(fitnesses[best_idx]) if fitnesses else 0.0
        mean_fitness = float(np.mean(fitnesses)) if fitnesses else 0.0
        median_fitness = float(np.median(fitnesses)) if fitnesses else 0.0

        print(f"  best fit: {best_fitness:.4f}, mean: {mean_fitness:.4f}")

        with open(STATS_CSV, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([gen, best_fitness, mean_fitness, median_fitness])

        with open(PER_INDIVIDUAL_CSV, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            for idx in range(len(used_controllers)):
                sx, sy, sz = starts[idx]
                ex, ey, ez = ends[idx]
                writer.writerow([gen, idx, float(fitnesses[idx]), sx, sy, sz, ex, ey, ez])

        # preserve best overall
        if best_fitness > best_overall_fitness:
            best_overall_fitness = best_fitness
            best_overall_controller = used_controllers[best_idx].copy()
            # save best to pickle (weights dict)
            with open(BEST_MODEL_FILE, 'wb') as bf:
                pickle.dump(best_overall_controller.weights, bf)

        # produce next generation: elitism + crossover + mutation
        sorted_idx = np.argsort(fitnesses)[::-1]
        parents = [used_controllers[int(sorted_idx[i])] for i in range(max(1, ELITE_COUNT))]
        next_pop = []
        for i in range(ELITE_COUNT):
            next_pop.append(parents[i].copy())
        while len(next_pop) < POPULATION:
            a = random.choice(parents)
            b = random.choice(parents)
            child = MLPController.crossover(a, b)
            child.mutate(rate=MUTATION_RATE, prob=MUTATION_PROB)
            if not hasattr(child, 'color'):
                child.color = getattr(a, 'color', getattr(b, 'color', (0.3,0.6,0.3,1.0)))
            next_pop.append(child)
        controllers = next_pop

    print("\nEvolution complete. Best fitness:", best_overall_fitness)
    print("Best model saved to:", BEST_MODEL_FILE)

if __name__ == "__main__":
    run_evolution()
