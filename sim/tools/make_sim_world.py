#!/usr/bin/env python3
"""Builds the Main Mission (Sim) Gazebo files from two sources, so neither
is copied by hand:

  sim/worlds/nidar_sim.sdf   Rish's NIDAR test maze (walls unchanged)
  ardupilot_gazebo           ArduPilot's iris_with_ardupilot model (SITL link)

Writes into OUT_DIR (default ~/nidar_sim/generated):
  models/nidar_iris_lidar/   iris_with_ardupilot + a 360 deg 2D LiDAR on top
                             (gz topic /scan, frame "laser" -- what the real
                             RPLIDAR A2 node publishes, so Cartographer runs
                             with the drone's own config)
  worlds/nidar_main_mission.sdf
                             the maze + the systems ArduPilot/the LiDAR need,
                             1 kHz physics, the drone at the entry (0, 0)
                             facing north, into the maze

    python3 sim/tools/make_sim_world.py --ardupilot-gazebo ~/nidar_sim/ardupilot_gazebo
"""
import argparse
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
MAZE = os.path.join(HERE, "..", "worlds", "nidar_sim.sdf")

# RPLIDAR A2-like: 360 deg, 0.5 deg steps, 10 Hz, 0.15-12 m.
LIDAR = """
    <link name="lidar_link">
      <pose>0 0 0.12 0 0 0</pose>
      <inertial>
        <mass>0.05</mass>
        <inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <visual name="lidar_visual">
        <geometry><cylinder><radius>0.035</radius><length>0.04</length></cylinder></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
      </visual>
      <sensor name="lidar" type="gpu_lidar">
        <topic>/scan</topic>
        <gz_frame_id>laser</gz_frame_id>
        <update_rate>10</update_rate>
        <always_on>1</always_on>
        <visualize>false</visualize>
        <lidar>
          <scan>
            <horizontal><samples>720</samples><resolution>1</resolution><min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal>
            <vertical><samples>1</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle></vertical>
          </scan>
          <range><min>0.15</min><max>12.0</max><resolution>0.01</resolution></range>
          <noise><type>gaussian</type><mean>0</mean><stddev>0.01</stddev></noise>
        </lidar>
      </sensor>
    </link>
    <joint name="lidar_joint" type="fixed">
      <parent>iris_with_standoffs::base_link</parent>
      <child>lidar_link</child>
    </joint>
"""

SYSTEMS = """
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
"""

DRONE = """
    <include>
      <uri>model://nidar_iris_lidar</uri>
      <name>nidar_drone</name>
      <pose degrees="true">0 0 0.195 0 0 90</pose>
    </include>
"""


def build_model(ardupilot_gazebo: str, out_dir: str) -> str:
    src = os.path.join(ardupilot_gazebo, "models", "iris_with_ardupilot", "model.sdf")
    text = open(src, encoding="utf-8").read()
    text = text.replace('<model name="iris_with_ardupilot">', '<model name="nidar_iris_lidar">', 1)
    if 'name="nidar_iris_lidar"' not in text:
        raise SystemExit(f"unexpected model name in {src}")
    i = text.rindex("</model>")
    text = text[:i] + LIDAR + text[i:]
    model_dir = os.path.join(out_dir, "models", "nidar_iris_lidar")
    os.makedirs(model_dir, exist_ok=True)
    with open(os.path.join(model_dir, "model.sdf"), "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)
    with open(os.path.join(model_dir, "model.config"), "w", encoding="utf-8", newline="\n") as fh:
        fh.write(
            '<?xml version="1.0"?>\n<model>\n  <name>NIDAR iris + 2D LiDAR</name>\n  <version>1.0</version>\n'
            '  <sdf version="1.9">model.sdf</sdf>\n  <description>ArduPilot iris_with_ardupilot with a 360 deg '
            "2D LiDAR (Main Mission (Sim))</description>\n</model>\n"
        )
    return model_dir


def build_world(out_dir: str) -> str:
    text = open(MAZE, encoding="utf-8").read()
    text = re.sub(r'<world name="[^"]+">', '<world name="nidar_main_mission">', text, count=1)
    text = re.sub(r"\s*<physics type=\"ode\">.*?</physics>", SYSTEMS, text, count=1, flags=re.S)
    i = text.rindex("</world>")
    text = text[:i] + DRONE + "  " + text[i:]
    worlds = os.path.join(out_dir, "worlds")
    os.makedirs(worlds, exist_ok=True)
    path = os.path.join(worlds, "nidar_main_mission.sdf")
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)
    return path


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ardupilot-gazebo", default=os.path.expanduser("~/nidar_sim/ardupilot_gazebo"))
    ap.add_argument("--out", default=os.path.expanduser("~/nidar_sim/generated"))
    args = ap.parse_args()
    print("model:", build_model(args.ardupilot_gazebo, args.out))
    print("world:", build_world(args.out))


if __name__ == "__main__":
    main()
