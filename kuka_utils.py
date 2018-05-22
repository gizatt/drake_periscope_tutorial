# -*- coding: utf8 -*-

import os.path
import numpy as np

import pydrake
from pydrake.all import (
    AddFlatTerrainToWorld,
    AddModelInstancesFromSdfString,
    AddModelInstanceFromUrdfFile,
    AddModelInstanceFromUrdfStringSearchingInRosPackages,
    FloatingBaseType,
    RigidBodyFrame,
)


def extract_position_indices(rbt, controlled_joint_names):
    ''' Given a RigidBodyTree and a list of
    joint names, returns, in separate lists, the
    position indices (i.e. offsets into the RBT positions vector)
    corresponding to those joints, and the rest of the
    position indices. '''
    controlled_config_inds = []
    other_config_inds = []
    for i in range(rbt.get_num_bodies()):
        body = rbt.get_body(i)
        if body.has_joint():
            joint = body.getJoint()
            if joint.get_name() in controlled_joint_names:
                controlled_config_inds += range(
                    body.get_position_start_index(),
                    body.get_position_start_index() +
                    joint.get_num_positions())
            else:
                other_config_inds += range(
                    body.get_position_start_index(),
                    body.get_position_start_index() +
                    joint.get_num_positions())
    if len(controlled_joint_names) != len(controlled_config_inds):
        raise ValueError("Didn't find all "
                         "requested controlled joint names.")

    return controlled_config_inds, other_config_inds


def setup_kuka(rbt):
    iiwa_urdf_path = os.path.join(
        pydrake.getDrakePath(),
        "manipulation", "models", "iiwa_description", "urdf",
        "iiwa14_polytope_collision.urdf")

    wsg50_sdf_path = os.path.join(
        pydrake.getDrakePath(),
        "manipulation", "models", "wsg_50_description", "sdf",
        "schunk_wsg_50.sdf")

    table_sdf_path = os.path.join(
        pydrake.getDrakePath(),
        "examples", "kuka_iiwa_arm", "models", "table",
        "extra_heavy_duty_table_surface_only_collision.sdf")

    object_urdf_path = os.path.join(
        pydrake.getDrakePath(),
        "examples", "kuka_iiwa_arm", "models", "objects",
        "block_for_pick_and_place.urdf")

    AddFlatTerrainToWorld(rbt)
    table_frame_robot = RigidBodyFrame(
        "table_frame_robot", rbt.world(),
        [0.0, 0, 0], [0, 0, 0])
    AddModelInstancesFromSdfString(
        open(table_sdf_path).read(), FloatingBaseType.kFixed,
        table_frame_robot, rbt)
    table_frame_fwd = RigidBodyFrame(
        "table_frame_fwd", rbt.world(),
        [0.8, 0, 0], [0, 0, 0])
    AddModelInstancesFromSdfString(
        open(table_sdf_path).read(), FloatingBaseType.kFixed,
        table_frame_fwd, rbt)

    table_top_z_in_world = 0.736 + 0.057 / 2

    robot_base_frame = RigidBodyFrame(
        "robot_base_frame", rbt.world(),
        [0.0, 0, table_top_z_in_world], [0, 0, 0])
    AddModelInstanceFromUrdfFile(iiwa_urdf_path, FloatingBaseType.kFixed,
                                 robot_base_frame, rbt)

    object_init_frame = RigidBodyFrame(
        "object_init_frame", rbt.world(),
        [0.8, 0, table_top_z_in_world+0.1], [0, 0, 0])
    AddModelInstanceFromUrdfFile(object_urdf_path,
                                 FloatingBaseType.kRollPitchYaw,
                                 object_init_frame, rbt)

    # Add gripper
    gripper_frame = rbt.findFrame("iiwa_frame_ee")
    AddModelInstancesFromSdfString(
        open(wsg50_sdf_path).read(), FloatingBaseType.kFixed,
        gripper_frame, rbt)


def render_system_with_graphviz(system, output_file="system_view.gz"):
    ''' Renders the Drake system (presumably a diagram,
    otherwise this graph will be fairly trivial) using
    graphviz to a specified file. '''
    from graphviz import Source
    string = system.GetGraphvizString()
    src = Source(string)
    src.render(output_file, view=False)
