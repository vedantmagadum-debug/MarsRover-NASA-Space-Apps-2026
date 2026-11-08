#!/usr/bin/env python3
"""Generate a Mars-like Gazebo Fortress world (SDF 1.8).

Features
  * reddish regolith ground, Mars gravity (3.71 m/s^2), butterscotch sky
  * gentle dunes (collision), colour patches (visual only), pebbles (visual only)
  * boulders: sphere clusters, tilted boxes, pillars (all with collision)
  * 3 craters with a broken rim of rocks and a darker floor
  * canyon walls on both sides and an end wall, so the lidar always has
    structure to map
  * a few guaranteed obstacles straight ahead of the spawn point

All collision obstacles are >= 0.45 m tall, because the 2D lidar sits about
0.29 m above the ground and cannot see anything lower than that.

Usage:
  python3 generate_world.py --out ../worlds/mars_world.sdf --seed 7 --rocks 40
"""
import argparse
import math
import random

ROCK_COLORS = [
    (0.30, 0.29, 0.29),
    (0.38, 0.36, 0.35),
    (0.24, 0.23, 0.25),
    (0.34, 0.31, 0.30),
    (0.28, 0.26, 0.24),
]
SAND = (0.72, 0.36, 0.19)
DUNE = (0.76, 0.40, 0.22)
CANYON = (0.45, 0.24, 0.14)

HEADER = """<?xml version="1.0"?>
<sdf version="1.8">
  <world name="mars_world">

    <physics name="mars_physics" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Mars gravity -->
    <gravity>0 0 -3.71</gravity>

    <plugin filename="ignition-gazebo-physics-system"
            name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system"
            name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system"
            name="ignition::gazebo::systems::SceneBroadcaster"/>
    <!-- renders camera + gpu_lidar. If your GPU/VM has trouble use: ogre -->
    <plugin filename="ignition-gazebo-sensors-system"
            name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <!-- without this the IMU never publishes -->
    <plugin filename="ignition-gazebo-imu-system"
            name="ignition::gazebo::systems::Imu"/>

    <scene>
      <ambient>0.55 0.38 0.30 1</ambient>
      <background>0.85 0.62 0.45 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 50 0 0 0</pose>
      <diffuse>1.0 0.82 0.65 1</diffuse>
      <specular>0.3 0.25 0.2 1</specular>
      <direction>-0.6 0.3 -0.7</direction>
    </light>

    <model name="mars_ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>140 140</size></plane></geometry>
          <surface>
            <friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>140 140</size></plane></geometry>
          <material>
            <ambient>0.55 0.27 0.15 1</ambient>
            <diffuse>0.72 0.36 0.19 1</diffuse>
            <specular>0.03 0.03 0.03 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""

FOOTER = """
  </world>
</sdf>
"""


def pose(x, y, z, r=0.0, p=0.0, yaw=0.0):
    return f"{x:.3f} {y:.3f} {z:.3f} {r:.3f} {p:.3f} {yaw:.3f}"


def sphere(r):
    return f"<sphere><radius>{r:.3f}</radius></sphere>"


def box(x, y, z):
    return f"<box><size>{x:.3f} {y:.3f} {z:.3f}</size></box>"


def cyl(r, length):
    return f"<cylinder><radius>{r:.3f}</radius><length>{length:.3f}</length></cylinder>"


def mat(c):
    r, g, b = c
    return (f"<material><ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>"
            f"<diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>"
            f"<specular>0.02 0.02 0.02 1</specular></material>")


class MarsWorld:
    def __init__(self, seed, n_rocks, x_min=4.0, x_max=32.0, half_width=8.0):
        self.rng = random.Random(seed)
        self.n_rocks = n_rocks
        self.x_min, self.x_max, self.half_width = x_min, x_max, half_width
        self.models = []
        self.placed = []  # (x, y, footprint radius)
        self.n_collision_models = 0

    # ---------------------------------------------------------------- helpers
    def add_model(self, name, p, shapes, collision=True):
        """shapes: list of (pose_str, geometry_xml, (r, g, b))."""
        out = [f'    <model name="{name}">',
               '      <static>true</static>',
               f'      <pose>{p}</pose>',
               '      <link name="link">']
        for i, (sp, geom, color) in enumerate(shapes):
            if collision:
                out.append(f'        <collision name="c{i}"><pose>{sp}</pose>'
                           f'<geometry>{geom}</geometry></collision>')
            out.append(f'        <visual name="v{i}"><pose>{sp}</pose>'
                       f'<geometry>{geom}</geometry>{mat(color)}</visual>')
        out.append('      </link>')
        out.append('    </model>')
        self.models.append("\n".join(out))
        if collision:
            self.n_collision_models += 1

    def free(self, x, y, fp, gap=1.1):
        if math.hypot(x, y) < 3.0 + fp:          # keep the spawn area clear
            return False
        for px, py, pf in self.placed:
            if math.hypot(x - px, y - py) < fp + pf + gap:
                return False
        return True

    # ----------------------------------------------------------------- rocks
    def rock(self, name, x, y, kind=None):
        rng = self.rng
        kind = kind or rng.choice(['cluster', 'cluster', 'box', 'pillar'])
        color = rng.choice(ROCK_COLORS)
        shapes, fp = [], 0.0
        if kind == 'cluster':
            base_r = rng.uniform(0.38, 0.85)
            for i in range(rng.randint(1, 3)):
                r = base_r if i == 0 else base_r * rng.uniform(0.6, 1.0)
                ox, oy = (0.0, 0.0) if i == 0 else (rng.uniform(-0.6, 0.6), rng.uniform(-0.6, 0.6))
                shapes.append((pose(ox, oy, r * 0.30), sphere(r), color))   # top at 1.3 r
                fp = max(fp, math.hypot(ox, oy) + r)
        elif kind == 'box':
            sx, sy, h = rng.uniform(0.6, 1.5), rng.uniform(0.6, 1.5), rng.uniform(0.6, 1.3)
            sp = pose(0, 0, h / 2 - 0.08, rng.uniform(-0.12, 0.12),
                      rng.uniform(-0.12, 0.12), rng.uniform(0, math.pi))
            shapes.append((sp, box(sx, sy, h), color))
            fp = math.hypot(sx, sy) / 2
        else:  # pillar
            r, h = rng.uniform(0.30, 0.55), rng.uniform(0.8, 1.6)
            sp = pose(0, 0, h / 2 - 0.08, rng.uniform(-0.08, 0.08), rng.uniform(-0.08, 0.08), 0)
            shapes.append((sp, cyl(r, h), color))
            fp = r
        self.add_model(name, pose(x, y, 0), shapes)
        self.placed.append((x, y, fp))

    def forced_obstacles(self):
        """Obstacles right in the rover's path so avoidance is always exercised."""
        self.rock("rock_ahead_1", 6.5, 0.0, 'cluster')
        self.rock("rock_ahead_2", 12.5, -0.8, 'box')
        self.rock("rock_ahead_3", 18.0, 0.9, 'pillar')
        self.rock("rock_ahead_4", 24.0, -0.3, 'cluster')

    def scatter_rocks(self):
        rng, placed_count, tries = self.rng, 0, 0
        while placed_count < self.n_rocks and tries < 4000:
            tries += 1
            x = rng.uniform(self.x_min, self.x_max)
            y = rng.uniform(-self.half_width, self.half_width)
            if self.free(x, y, 0.8):
                self.rock(f"rock_{placed_count}", x, y)
                placed_count += 1
        return placed_count

    # --------------------------------------------------------------- craters
    def crater(self, name, cx, cy, rc):
        rng = self.rng
        self.add_model(f"{name}_floor", pose(cx, cy, 0.004),
                       [(pose(0, 0, 0), cyl(rc, 0.004), (0.38, 0.18, 0.10))],
                       collision=False)
        n = 16
        gaps = set(rng.sample(range(n), 3))
        color = rng.choice(ROCK_COLORS)
        shapes = []
        for i in range(n):
            if i in gaps:
                continue
            a = 2 * math.pi * i / n + rng.uniform(-0.08, 0.08)
            r = rng.uniform(0.38, 0.55)
            shapes.append((pose(rc * math.cos(a), rc * math.sin(a), r * 0.30), sphere(r), color))
        self.add_model(f"{name}_rim", pose(cx, cy, 0), shapes)
        self.placed.append((cx, cy, rc + 0.8))

    # ------------------------------------------------------------------ dunes
    def dunes(self, n=4):
        rng = self.rng
        for i in range(n):
            R = rng.uniform(30.0, 45.0)
            h = rng.uniform(0.10, 0.15)            # very gentle: ~5 deg max slope
            x = rng.uniform(9.0, 28.0)
            y = rng.uniform(-6.0, 6.0)
            self.add_model(f"dune_{i}", pose(x, y, h - R),
                           [(pose(0, 0, 0), sphere(R), DUNE)])

    # --------------------------------------------------------------- scenery
    def patches(self, n=36):
        rng = self.rng
        shapes = []
        for i in range(n):
            s = rng.uniform(0.82, 1.18)
            color = (min(SAND[0] * s, 1), SAND[1] * s * rng.uniform(0.9, 1.1), SAND[2] * s)
            shapes.append((pose(rng.uniform(-8, 38), rng.uniform(-11, 11), 0.003 + i * 0.0003),
                           cyl(rng.uniform(1.0, 4.0), 0.002), color))
        self.add_model("terrain_patches", pose(0, 0, 0), shapes, collision=False)

    def pebbles(self, n=160):
        rng = self.rng
        shapes = []
        for _ in range(n):
            r = rng.uniform(0.02, 0.05)
            shapes.append((pose(rng.uniform(-6, 36), rng.uniform(-10, 10), r * 0.4),
                           sphere(r), rng.choice(ROCK_COLORS)))
        self.add_model("pebbles", pose(0, 0, 0), shapes, collision=False)

    def canyon(self):
        rng = self.rng
        x, i = -7.0, 0
        while x < 36.0:
            for side in (-1, 1):
                h = rng.uniform(1.2, 2.2)
                length = 5.2
                y = side * (10.5 + rng.uniform(-0.3, 0.5))
                self.add_model(f"wall_{i}_{'l' if side > 0 else 'r'}",
                               pose(x + length / 2, y, h / 2 - 0.1, 0, 0, rng.uniform(-0.06, 0.06)),
                               [(pose(0, 0, 0), box(length, 1.2, h), CANYON)])
            x += 5.0
            i += 1
        self.add_model("wall_end", pose(37.0, 0, 0.8), [(pose(0, 0, 0), box(1.2, 23.0, 1.8), CANYON)])
        self.add_model("wall_back", pose(-7.5, 0, 0.8), [(pose(0, 0, 0), box(1.2, 23.0, 1.8), CANYON)])

    # ----------------------------------------------------------------- build
    def build(self):
        self.canyon()
        self.forced_obstacles()
        self.crater("crater_a", 10.0, 5.2, 2.8)
        self.crater("crater_b", 21.0, -5.2, 3.2)
        self.crater("crater_c", 28.0, 4.2, 2.5)
        n = self.scatter_rocks()
        self.dunes()
        self.patches()
        self.pebbles()
        return HEADER + "\n" + "\n".join(self.models) + FOOTER, n


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--out', default='mars_world.sdf')
    ap.add_argument('--seed', type=int, default=7)
    ap.add_argument('--rocks', type=int, default=40, help='random boulders (in addition to the fixed ones)')
    args = ap.parse_args()

    world = MarsWorld(args.seed, args.rocks)
    sdf, n = world.build()
    with open(args.out, 'w') as fh:
        fh.write(sdf)
    print(f"wrote {args.out}: {n} random boulders, {len(world.models)} models "
          f"({world.n_collision_models} with collision), seed={args.seed}")


if __name__ == '__main__':
    main()
