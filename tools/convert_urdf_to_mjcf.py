#!/usr/bin/env python3
"""Convert the nchex ROS/Gazebo URDF into a standalone MuJoCo MJCF model.

The converter deliberately does not copy Gazebo plugins.  It translates the
kinematic tree, inertials, visual/collision geometry, Delta closure joints,
three Delta position actuators, and six rotor thrust actuators.
"""

from __future__ import annotations

import argparse
import math
import shutil
from collections import defaultdict
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_URDF = (
    PROJECT_DIR.parent
    / "ros1"
    / "模型文件"
    / "nchex"
    / "urdf"
    / "nchex_rotors.urdf"
)
DEFAULT_MESH_DIR = DEFAULT_URDF.parents[1] / "meshes"
DEFAULT_OUTPUT = PROJECT_DIR / "model" / "nchex.xml"

ROTOR_REACTION_SIGNS = (-1, 1, 1, -1, 1, -1)
ROTOR_MOMENT_CONSTANT = 0.06
DELTA_MOTOR_JOINTS = ("delta_motor1", "delta_motor2", "delta_motor3")


def floats(text: str | None, default: tuple[float, ...]) -> tuple[float, ...]:
    if not text:
        return default
    return tuple(float(value) for value in text.split())


def fmt(values) -> str:
    return " ".join(f"{float(value):.12g}" for value in values)


def rpy_matrix(rpy: tuple[float, float, float]) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        ),
        dtype=float,
    )


def matrix_quat(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Return a MuJoCo quaternion in w, x, y, z order."""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        axis = int(np.argmax(np.diag(rotation)))
        if axis == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            w = (rotation[2, 1] - rotation[1, 2]) / scale
            x = 0.25 * scale
            y = (rotation[0, 1] + rotation[1, 0]) / scale
            z = (rotation[0, 2] + rotation[2, 0]) / scale
        elif axis == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            w = (rotation[0, 2] - rotation[2, 0]) / scale
            x = (rotation[0, 1] + rotation[1, 0]) / scale
            y = 0.25 * scale
            z = (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            w = (rotation[1, 0] - rotation[0, 1]) / scale
            x = (rotation[0, 2] + rotation[2, 0]) / scale
            y = (rotation[1, 2] + rotation[2, 1]) / scale
            z = 0.25 * scale
    quaternion = np.array((w, x, y, z), dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    return tuple(float(value) for value in quaternion)


def origin_transform(origin: ET.Element | None) -> np.ndarray:
    transform = np.eye(4)
    if origin is None:
        return transform
    transform[:3, :3] = rpy_matrix(floats(origin.get("rpy"), (0.0, 0.0, 0.0)))
    transform[:3, 3] = floats(origin.get("xyz"), (0.0, 0.0, 0.0))
    return transform


def transform_attributes(origin: ET.Element | None) -> dict[str, str]:
    transform = origin_transform(origin)
    return {
        "pos": fmt(transform[:3, 3]),
        "quat": fmt(matrix_quat(transform[:3, :3])),
    }


def sanitize(name: str) -> str:
    return "".join(character if character.isalnum() else "_" for character in name)


def convert_dae_to_obj(source: Path, destination: Path) -> None:
    """Convert the simple indexed COLLADA mesh used by this project to OBJ.

    The source file was exported with an incorrect Y_UP annotation.  Gazebo's
    existing URDF offsets assume the raw XYZ coordinates, so no axis swap is
    applied here.
    """
    root = ET.parse(source).getroot()
    namespace = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    mesh = root.find(".//c:geometry/c:mesh", namespace)
    if mesh is None:
        raise ValueError(f"No COLLADA mesh found in {source}")

    sources: dict[str, list[tuple[float, ...]]] = {}
    for item in mesh.findall("c:source", namespace):
        array = item.find("c:float_array", namespace)
        accessor = item.find("c:technique_common/c:accessor", namespace)
        if array is None or accessor is None:
            continue
        stride = int(accessor.get("stride", "1"))
        values = [float(value) for value in (array.text or "").split()]
        sources[item.get("id", "")] = [
            tuple(values[index : index + stride])
            for index in range(0, len(values), stride)
        ]

    vertices_map: dict[str, str] = {}
    for vertices in mesh.findall("c:vertices", namespace):
        position = vertices.find("c:input[@semantic='POSITION']", namespace)
        if position is not None:
            vertices_map[vertices.get("id", "")] = position.get("source", "").lstrip("#")

    output_vertices: list[tuple[float, ...]] = []
    output_faces: list[tuple[int, int, int]] = []
    vertex_offset = 0
    for triangles in mesh.findall("c:triangles", namespace):
        inputs = triangles.findall("c:input", namespace)
        stride = max(int(item.get("offset", "0")) for item in inputs) + 1
        vertex_input = next(item for item in inputs if item.get("semantic") == "VERTEX")
        vertex_slot = int(vertex_input.get("offset", "0"))
        source_id = vertices_map[vertex_input.get("source", "").lstrip("#")]
        positions = sources[source_id]
        output_vertices.extend(positions)
        indices = [int(value) for value in (triangles.findtext("c:p", "", namespace)).split()]
        corners = [indices[index + vertex_slot] + 1 + vertex_offset for index in range(0, len(indices), stride)]
        output_faces.extend(tuple(corners[index : index + 3]) for index in range(0, len(corners), 3))
        vertex_offset += len(positions)

    lines = ["# Converted from fixed_base2.dae for MuJoCo"]
    lines.extend(f"v {fmt(vertex[:3])}" for vertex in output_vertices)
    lines.extend(f"f {a} {b} {c}" for a, b, c in output_faces)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")


class Converter:
    def __init__(self, urdf_path: Path, source_mesh_dir: Path, output_path: Path):
        self.urdf_path = urdf_path
        self.source_mesh_dir = source_mesh_dir
        self.output_path = output_path
        self.output_mesh_dir = PROJECT_DIR / "meshes"
        self.urdf = ET.parse(urdf_path).getroot()
        self.links = {link.get("name"): link for link in self.urdf.findall("link")}
        self.joints = {joint.get("name"): joint for joint in self.urdf.findall("joint")}
        self.children: dict[str, list[ET.Element]] = defaultdict(list)
        self.parent_joint: dict[str, ET.Element] = {}
        for joint in self.joints.values():
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            self.children[parent].append(joint)
            self.parent_joint[child] = joint
        roots = sorted(set(self.links) - set(self.parent_joint))
        if roots != ["base_link"]:
            raise ValueError(f"Expected base_link as the only root, got {roots}")
        self.mesh_assets: dict[tuple[str, tuple[float, ...]], str] = {}
        self.body_elements: dict[str, ET.Element] = {}
        self.world_transforms: dict[str, np.ndarray] = {"base_link": np.eye(4)}

    def prepare_meshes(self) -> None:
        if self.output_mesh_dir.exists():
            shutil.rmtree(self.output_mesh_dir)
        shutil.copytree(self.source_mesh_dir, self.output_mesh_dir)
        dae = self.output_mesh_dir / "delta" / "fixed_base2.dae"
        convert_dae_to_obj(dae, dae.with_suffix(".obj"))

    def mesh_asset(self, filename: str, scale: tuple[float, ...], asset_root: ET.Element) -> str:
        relative = filename.split("/meshes/", 1)[-1]
        if relative.lower().endswith(".dae"):
            relative = str(Path(relative).with_suffix(".obj"))
        relative = relative.replace("\\", "/")
        key = (relative, scale)
        if key in self.mesh_assets:
            return self.mesh_assets[key]
        name = f"mesh_{len(self.mesh_assets)}_{sanitize(Path(relative).stem)}"
        ET.SubElement(asset_root, "mesh", name=name, file=relative, scale=fmt(scale))
        self.mesh_assets[key] = name
        return name

    def add_inertial(self, body: ET.Element, link: ET.Element) -> None:
        inertial = link.find("inertial")
        if inertial is None:
            return
        inertia = inertial.find("inertia")
        mass = float(inertial.find("mass").get("value"))
        tensor = np.array(
            (
                (float(inertia.get("ixx")), float(inertia.get("ixy", "0")), float(inertia.get("ixz", "0"))),
                (float(inertia.get("ixy", "0")), float(inertia.get("iyy")), float(inertia.get("iyz", "0"))),
                (float(inertia.get("ixz", "0")), float(inertia.get("iyz", "0")), float(inertia.get("izz"))),
            ),
            dtype=float,
        )
        transform = origin_transform(inertial.find("origin"))
        # MuJoCo does not allow an orientation together with fullinertia. Rotate
        # the URDF tensor into the link frame and then omit the inertial quat.
        tensor = transform[:3, :3] @ tensor @ transform[:3, :3].T
        values = (
            tensor[0, 0], tensor[1, 1], tensor[2, 2],
            tensor[0, 1], tensor[0, 2], tensor[1, 2],
        )
        attributes = {"pos": fmt(transform[:3, 3])}
        attributes.update(mass=f"{mass:.12g}", fullinertia=fmt(values))
        ET.SubElement(body, "inertial", attributes)

    def visual_color(self, link_name: str, visual: ET.Element) -> str:
        material = visual.find("material")
        if material is not None:
            color = material.find("color")
            if color is not None:
                return color.get("rgba")
            named = self.urdf.find(f"material[@name='{material.get('name')}']/color")
            if named is not None:
                return named.get("rgba")
        if link_name.startswith("delta_"):
            return "0.82 0.08 0.06 1"
        if link_name.startswith("rotor_"):
            return "0.85 0.85 0.88 1"
        if link_name.startswith("motor_"):
            return "0.28 0.30 0.33 1"
        return "0.72 0.74 0.78 1"

    def add_geometry(
        self,
        body: ET.Element,
        geometry_parent: ET.Element,
        link_name: str,
        index: int,
        visual: bool,
        asset_root: ET.Element,
    ) -> None:
        geometry = geometry_parent.find("geometry")
        if geometry is None or len(geometry) != 1:
            return
        shape = geometry[0]
        attributes = transform_attributes(geometry_parent.find("origin"))
        attributes["name"] = f"{sanitize(link_name)}_{'visual' if visual else 'collision'}_{index}"
        if visual:
            attributes.update(
                contype="0",
                conaffinity="0",
                group="2",
                rgba=self.visual_color(link_name, geometry_parent),
            )
        else:
            attributes.update(
                contype="1",
                conaffinity="1",
                group="3",
                rgba="0.2 0.4 0.8 0",
                friction="0.9 0.02 0.001",
            )

        if shape.tag == "mesh":
            scale = floats(shape.get("scale"), (1.0, 1.0, 1.0))
            attributes.update(
                type="mesh",
                mesh=self.mesh_asset(shape.get("filename"), scale, asset_root),
            )
        elif shape.tag == "box":
            size = np.array(floats(shape.get("size"), (1.0, 1.0, 1.0))) / 2.0
            attributes.update(type="box", size=fmt(size))
        elif shape.tag == "cylinder":
            radius = float(shape.get("radius"))
            half_length = float(shape.get("length")) / 2.0
            attributes.update(type="cylinder", size=fmt((radius, half_length)))
        elif shape.tag == "sphere":
            attributes.update(type="sphere", size=shape.get("radius"))
        else:
            raise ValueError(f"Unsupported URDF geometry {shape.tag}")
        ET.SubElement(body, "geom", attributes)

    def add_link_contents(self, body: ET.Element, link_name: str, asset_root: ET.Element) -> None:
        link = self.links[link_name]
        self.add_inertial(body, link)
        for index, visual in enumerate(link.findall("visual")):
            self.add_geometry(body, visual, link_name, index, True, asset_root)
        for index, collision in enumerate(link.findall("collision")):
            self.add_geometry(body, collision, link_name, index, False, asset_root)
        if link_name == "base_link":
            ET.SubElement(body, "site", name="imu_site", pos="0 0 0", size="0.008", rgba="0 0.8 0 1")
        if link_name.startswith("motor_") and link_name.endswith("_link"):
            rotor_index = int(link_name.split("_")[1])
            # Apply the rotor wrench to the fixed motor housing at the rotor
            # centre. Applying reaction torque to the freely spinning propeller
            # link would accelerate only that visual joint.
            ET.SubElement(
                body,
                "site",
                name=f"rotor_{rotor_index}_thrust_site",
                pos="0 0 0.033",
                quat="1 0 0 0",
                size="0.006",
                rgba="0 0.5 1 1",
            )

    def add_joint(self, body: ET.Element, joint: ET.Element) -> None:
        joint_type = joint.get("type")
        if joint_type == "fixed":
            return
        attributes = {
            "name": joint.get("name"),
            "type": "slide" if joint_type == "prismatic" else "hinge",
            "pos": "0 0 0",
            "axis": joint.find("axis").get("xyz", "1 0 0") if joint.find("axis") is not None else "1 0 0",
        }
        dynamics = joint.find("dynamics")
        if dynamics is not None:
            attributes["damping"] = dynamics.get("damping", "0")
            attributes["frictionloss"] = dynamics.get("friction", "0")
        limit = joint.find("limit")
        if joint_type in ("revolute", "prismatic") and limit is not None:
            attributes["range"] = f"{limit.get('lower')} {limit.get('upper')}"
            attributes["limited"] = "true"
        ET.SubElement(body, "joint", attributes)

    def add_body_tree(self, parent_xml: ET.Element, link_name: str, asset_root: ET.Element) -> None:
        if link_name == "base_link":
            body = ET.SubElement(parent_xml, "body", name=link_name, pos="0 0 0.62")
            ET.SubElement(body, "freejoint", name="base_freejoint")
        else:
            joint = self.parent_joint[link_name]
            attributes = {"name": link_name, **transform_attributes(joint.find("origin"))}
            body = ET.SubElement(parent_xml, "body", attributes)
            self.add_joint(body, joint)
            parent_link = joint.find("parent").get("link")
            self.world_transforms[link_name] = self.world_transforms[parent_link] @ origin_transform(joint.find("origin"))
        self.body_elements[link_name] = body
        self.add_link_contents(body, link_name, asset_root)
        for child_joint in sorted(self.children.get(link_name, []), key=lambda item: item.get("name")):
            child_name = child_joint.find("child").get("link")
            self.add_body_tree(body, child_name, asset_root)

    def add_closure_constraints(self, equality: ET.Element) -> None:
        epsilon = 0.025
        for gazebo in self.urdf.findall("gazebo"):
            for closure in gazebo.findall("joint"):
                name = closure.get("name")
                parent_name = closure.findtext("parent").strip()
                child_name = closure.findtext("child").strip()
                pose_values = floats(closure.findtext("pose"), (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
                anchor_child = np.array(pose_values[:3], dtype=float)
                axis_joint = np.array(floats(closure.findtext("axis/xyz"), (1.0, 0.0, 0.0)), dtype=float)
                axis_child = rpy_matrix(tuple(pose_values[3:])) @ axis_joint
                axis_child /= np.linalg.norm(axis_child)

                child_world = self.world_transforms[child_name]
                parent_world = self.world_transforms[parent_name]
                points_child = (anchor_child, anchor_child + epsilon * axis_child)
                for point_index, child_point in enumerate(points_child):
                    point_world = child_world @ np.array((*child_point, 1.0))
                    parent_point = np.linalg.inv(parent_world) @ point_world
                    # MuJoCo 2.3 (used by the ramps environment) supports the
                    # body/anchor form of connect, but not the newer site1/site2
                    # form. Two point connections along the axis leave exactly
                    # the intended revolute degree of freedom.
                    ET.SubElement(
                        equality,
                        "connect",
                        name=f"{name}_constraint_{point_index}",
                        body1=parent_name,
                        body2=child_name,
                        anchor=fmt(parent_point[:3]),
                        solref="0.004 1",
                        solimp="0.95 0.99 0.001",
                    )

    def convert(self) -> None:
        self.prepare_meshes()
        model = ET.Element("mujoco", model="nchex")
        ET.SubElement(
            model,
            "compiler",
            angle="radian",
            meshdir="../meshes",
            autolimits="true",
            balanceinertia="true",
            discardvisual="false",
            fusestatic="false",
        )
        ET.SubElement(
            model,
            "option",
            timestep="0.002",
            gravity="0 0 -9.81",
            integrator="implicitfast",
            solver="Newton",
            iterations="80",
        )
        visual = ET.SubElement(model, "visual")
        ET.SubElement(visual, "headlight", ambient="0.35 0.35 0.35", diffuse="0.75 0.75 0.75", specular="0.2 0.2 0.2")
        ET.SubElement(visual, "global", azimuth="135", elevation="-22")

        defaults = ET.SubElement(model, "default")
        ET.SubElement(defaults, "joint", damping="0.001", armature="1e-6")
        ET.SubElement(defaults, "geom", solref="0.01 1", solimp="0.9 0.95 0.001")

        asset = ET.SubElement(model, "asset")
        ET.SubElement(asset, "texture", name="sky", type="skybox", builtin="gradient", rgb1="0.35 0.45 0.60", rgb2="0.02 0.02 0.03", width="512", height="3072")
        ET.SubElement(asset, "texture", name="ground_tex", type="2d", builtin="checker", rgb1="0.28 0.30 0.32", rgb2="0.18 0.20 0.22", width="512", height="512")
        ET.SubElement(asset, "material", name="ground_mat", texture="ground_tex", texrepeat="8 8", reflectance="0.15")

        worldbody = ET.SubElement(model, "worldbody")
        ET.SubElement(worldbody, "light", name="sun", pos="0 -2 4", dir="0 0 -1", directional="true")
        ET.SubElement(worldbody, "geom", name="floor", type="plane", size="4 4 0.1", material="ground_mat", friction="1.0 0.02 0.001")
        self.add_body_tree(worldbody, "base_link", asset)

        contact = ET.SubElement(model, "contact")
        for joint in self.joints.values():
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            ET.SubElement(contact, "exclude", body1=parent, body2=child)
        for index in range(6):
            ET.SubElement(contact, "exclude", body1="base_link", body2=f"rotor_{index}_link")

        equality = ET.SubElement(model, "equality")
        self.add_closure_constraints(equality)

        actuator = ET.SubElement(model, "actuator")
        for index, sign in enumerate(ROTOR_REACTION_SIGNS):
            ET.SubElement(
                actuator,
                "general",
                name=f"rotor_{index}_thrust",
                site=f"rotor_{index}_thrust_site",
                gear=fmt((0.0, 0.0, 1.0, 0.0, 0.0, sign * ROTOR_MOMENT_CONSTANT)),
                ctrlrange="0 30",
                forcerange="0 30",
            )
        for joint_name in DELTA_MOTOR_JOINTS:
            ET.SubElement(
                actuator,
                "position",
                name=f"{joint_name}_position",
                joint=joint_name,
                kp="80",
                ctrlrange="-1.1 1.1",
                forcerange="-20 20",
            )

        sensor = ET.SubElement(model, "sensor")
        ET.SubElement(sensor, "framequat", name="base_orientation", objtype="site", objname="imu_site")
        ET.SubElement(sensor, "gyro", name="base_angular_velocity", site="imu_site")
        ET.SubElement(sensor, "accelerometer", name="base_linear_acceleration", site="imu_site")

        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        ET.indent(model, space="  ")
        ET.ElementTree(model).write(self.output_path, encoding="utf-8", xml_declaration=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--mesh-dir", type=Path, default=DEFAULT_MESH_DIR)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    converter = Converter(arguments.urdf.resolve(), arguments.mesh_dir.resolve(), arguments.output.resolve())
    converter.convert()
    print(f"Generated {arguments.output.resolve()}")
    print(f"Copied MuJoCo meshes to {(PROJECT_DIR / 'meshes').resolve()}")


if __name__ == "__main__":
    main()
