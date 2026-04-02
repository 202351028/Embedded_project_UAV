from pathlib import Path
from typing import List

from .config_loader import load_swarm_config
from .models import ZoneConfig


def _fmt_pose(x: float, y: float, z: float) -> str:
    return f"{x:.3f} {y:.3f} {z:.3f} 0 0 0"


def _zone_model_xml(zone: ZoneConfig) -> str:
    x = float(zone.center_ned_m[0])
    y = float(zone.center_ned_m[1])
    h = max(0.2, zone.height_m)
    pose_z = h * 0.5

    if zone.kind == "obstacle":
        amb = "0.74 0.22 0.16 0.95"
        dif = "0.74 0.22 0.16 0.95"
    else:
        amb = "0.96 0.33 0.14 0.55"
        dif = "0.96 0.33 0.14 0.55"

    return f"""
    <model name=\"zone_{zone.name}\">
      <static>true</static>
      <pose>{_fmt_pose(x, y, pose_z)}</pose>
      <link name=\"link\">
        <collision name=\"collision\">
          <geometry><cylinder><radius>{zone.radius_m:.3f}</radius><length>{h:.3f}</length></cylinder></geometry>
        </collision>
        <visual name=\"visual\">
          <geometry><cylinder><radius>{zone.radius_m:.3f}</radius><length>{h:.3f}</length></cylinder></geometry>
          <material>
            <ambient>{amb}</ambient>
            <diffuse>{dif}</diffuse>
          </material>
        </visual>
      </link>
    </model>
    """.rstrip()


def build_world_xml(config_path: Path) -> str:
    cfg = load_swarm_config(config_path)
    leader = next(d for d in cfg.drone_cfgs if d.name == cfg.leader_name)

    source_x = float(leader.source_ned_m[0])
    source_y = float(leader.source_ned_m[1])
    dest_x = float(leader.destination_ned_m[0])
    dest_y = float(leader.destination_ned_m[1])

    zone_xml: List[str] = [_zone_model_xml(z) for z in cfg.zones]

    return f"""<?xml version=\"1.0\" ?>
<sdf version=\"1.6\">
  <world name=\"swarm_city_realworld\">
    <include>
      <uri>model://sun</uri>
    </include>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <scene>
      <ambient>0.68 0.68 0.68 1</ambient>
      <background>0.82 0.90 0.96 1</background>
      <shadows>true</shadows>
    </scene>

    <physics type=\"ode\">
      <max_step_size>0.004</max_step_size>
      <real_time_update_rate>250</real_time_update_rate>
      <real_time_factor>1</real_time_factor>
    </physics>

    <model name=\"source_marker\">
      <static>true</static>
      <pose>{_fmt_pose(source_x, source_y, 0.03)}</pose>
      <link name=\"link\">
        <visual name=\"disc\">
          <geometry><cylinder><radius>2.1</radius><length>0.06</length></cylinder></geometry>
          <material>
            <ambient>0.10 0.70 0.20 0.85</ambient>
            <diffuse>0.10 0.70 0.20 0.85</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name=\"destination_marker\">
      <static>true</static>
      <pose>{_fmt_pose(dest_x, dest_y, 0.03)}</pose>
      <link name=\"link\">
        <visual name=\"disc\">
          <geometry><cylinder><radius>2.4</radius><length>0.06</length></cylinder></geometry>
          <material>
            <ambient>0.10 0.34 0.88 0.85</ambient>
            <diffuse>0.10 0.34 0.88 0.85</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name=\"leader_start_ring\">
      <static>true</static>
      <pose>{_fmt_pose(source_x, source_y, 0.08)}</pose>
      <link name=\"outer\">
        <visual name=\"outer_visual\">
          <geometry><cylinder><radius>3.2</radius><length>0.05</length></cylinder></geometry>
          <material>
            <ambient>0.92 0.74 0.08 0.45</ambient>
            <diffuse>0.92 0.74 0.08 0.45</diffuse>
          </material>
        </visual>
      </link>
    </model>

{chr(10).join(zone_xml)}
  </world>
</sdf>
"""


def write_world_file(config_path: Path, world_output: Path) -> None:
    xml = build_world_xml(config_path)
    world_output.parent.mkdir(parents=True, exist_ok=True)
    world_output.write_text(xml, encoding="utf-8")
