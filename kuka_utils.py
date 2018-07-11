# -*- coding: utf8 -*-

import os.path
from matplotlib import cm
import numpy as np

import pydrake
from pydrake.all import (
    AddFlatTerrainToWorld,
    AddModelInstancesFromSdfString,
    AddModelInstanceFromUrdfFile,
    AddModelInstanceFromUrdfStringSearchingInRosPackages,
    FloatingBaseType,
    LeafSystem,
    PortDataType,
    RigidBodyFrame,
    RollPitchYaw,
    RotationMatrix
)
from pydrake.solvers import ik

import meshcat
import meshcat.transformations as tf
import meshcat.geometry as g


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


_table_top_z_in_world = 0.736 + 0.057 / 2
_manipuland_body_indices = []


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

    robot_base_frame = RigidBodyFrame(
        "robot_base_frame", rbt.world(),
        [0.0, 0, _table_top_z_in_world], [0, 0, 0])
    AddModelInstanceFromUrdfFile(iiwa_urdf_path, FloatingBaseType.kFixed,
                                 robot_base_frame, rbt)

    # Add gripper
    gripper_frame = rbt.findFrame("iiwa_frame_ee")
    AddModelInstancesFromSdfString(
        open(wsg50_sdf_path).read(), FloatingBaseType.kFixed,
        gripper_frame, rbt)


def add_block_to_tabletop(rbt):
    object_urdf_path = os.path.join(
        pydrake.getDrakePath(),
        "examples", "kuka_iiwa_arm", "models", "objects",
        "block_for_pick_and_place.urdf")

    object_init_frame = RigidBodyFrame(
        "object_init_frame", rbt.world(),
        [0.8, 0, _table_top_z_in_world+0.1], [0, 0, 0])

    AddModelInstanceFromUrdfFile(object_urdf_path,
                                 FloatingBaseType.kRollPitchYaw,
                                 object_init_frame, rbt)
    _manipuland_body_indices.append(rbt.get_num_bodies()-1)


def add_cut_cylinders_to_tabletop(rbt, n_objects, do_convex_decomp=False):
    import mesh_creation
    import trimesh
    for k in range(n_objects):
        # Determine parameters of the cylinders
        height = np.random.random() * 0.03 + 0.04
        radius = np.random.random() * 0.02 + 0.01
        cut_dir = np.random.random(3)-0.5
        cut_dir[2] = 0.
        cut_dir /= np.linalg.norm(cut_dir)
        cut_point = (np.random.random(3) - 0.5)*radius*1.5
        cutting_planes = [(cut_point, cut_dir)]

        # Create a mesh programmatically for that cylinder
        cyl = mesh_creation.create_cut_cylinder(
            radius, height, cutting_planes, sections=20)
        cyl.density = 1000.  # Same as water
        init_pos = [0.6 + np.random.random()*0.2,
                    -0.2 + np.random.random()*0.4,
                    _table_top_z_in_world+radius+0.001]
        init_rot = [0., np.pi/2., np.random.random() * np.pi * 2.]

        # Save it out to a file and add it to the RBT
        object_init_frame = RigidBodyFrame(
            "object_init_frame_%f" % k, rbt.world(),
            init_pos, init_rot)

        if do_convex_decomp:  # more powerful, does a convex decomp
            urdf_dir = "/tmp/mesh_%d/" % k
            trimesh.io.urdf.export_urdf(cyl, urdf_dir)
            urdf_path = urdf_dir + "mesh_%d.urdf" % k
            AddModelInstanceFromUrdfFile(urdf_path,
                                         FloatingBaseType.kRollPitchYaw,
                                         object_init_frame, rbt)
            _manipuland_body_indices.append(rbt.get_num_bodies()-1)
        else:
            sdf_dir = "/tmp/mesh_%d/" % k
            name = "mesh_%d" % k
            mesh_creation.export_sdf(
                cyl, name, sdf_dir, color=[0.75, 0.2, 0.2, 1.])
            sdf_path = sdf_dir + "mesh_%d.sdf" % k
            AddModelInstancesFromSdfString(
                open(sdf_path).read(), FloatingBaseType.kRollPitchYaw,
                object_init_frame, rbt)
            _manipuland_body_indices.append(rbt.get_num_bodies()-1)


def project_rbt_to_nearest_feasible_on_table(rbt, q0):
    # Project arrangement to nonpenetration with IK
    constraints = []

    constraints.append(ik.MinDistanceConstraint(
        model=rbt, min_distance=0.01, active_bodies_idx=list(),
        active_group_names=set()))

    locked_position_inds = []
    for body_i in range(rbt.get_num_bodies()):
        if body_i in _manipuland_body_indices:
            constraints.append(ik.WorldPositionConstraint(
                model=rbt, body=body_i,
                pts=np.array([0., 0., 0.]),
                lb=np.array([0.6, -0.2, _table_top_z_in_world]),
                ub=np.array([1.0, 0.2, _table_top_z_in_world+0.3])))
        else:
            body = rbt.get_body(body_i)
            if body.has_joint():
                for k in range(body.get_position_start_index(),
                               body.get_position_start_index() +
                               body.getJoint().get_num_positions()):
                    locked_position_inds.append(k)

    required_posture_constraint = ik.PostureConstraint(rbt)
    required_posture_constraint.setJointLimits(
        locked_position_inds, q0[locked_position_inds]-0.001,
        q0[locked_position_inds]+0.001)
    constraints.append(required_posture_constraint)

    options = ik.IKoptions(rbt)
    options.setMajorIterationsLimit(10000)
    options.setIterationsLimit(100000)
    results = ik.InverseKin(
        rbt, q0, q0, constraints, options)

    qf = results.q_sol[0]
    info = results.info[0]
    print "Projected to feasibility with info %d" % info
    return qf


def render_system_with_graphviz(system, output_file="system_view.gz"):
    ''' Renders the Drake system (presumably a diagram,
    otherwise this graph will be fairly trivial) using
    graphviz to a specified file. '''
    from graphviz import Source
    string = system.GetGraphvizString()
    src = Source(string)
    src.render(output_file, view=False)


class RgbdCameraMeshcatVisualizer(LeafSystem):
    def __init__(self,
                 camera,
                 rbt,
                 draw_timestep=0.033333,
                 prefix="RBCameraViz",
                 zmq_url="tcp://127.0.0.1:6000"):
        LeafSystem.__init__(self)
        self.set_name('camera meshcat visualization')
        self.timestep = draw_timestep
        self._DeclarePeriodicPublish(draw_timestep, 0.0)
        self.camera = camera
        self.rbt = rbt
        self.prefix = prefix

        self.camera_input_port = \
            self._DeclareInputPort(PortDataType.kAbstractValued,
                                   camera.depth_image_output_port().size())
        self.state_input_port = \
            self._DeclareInputPort(PortDataType.kVectorValued,
                                   rbt.get_num_positions() +
                                   rbt.get_num_velocities())

        # Set up meshcat
        self.vis = meshcat.Visualizer(zmq_url=zmq_url)
        self.vis[prefix].delete()

    def _DoPublish(self, context, event):
        u_data = self.EvalAbstractInput(context, 0).get_value()
        x = self.EvalVectorInput(context, 1).get_value()
        w, h, _ = u_data.data.shape
        depth_image = u_data.data[:, :, 0]

        # Convert depth image to point cloud, with +z being
        # camera "forward"
        Kinv = np.linalg.inv(
            self.camera.depth_camera_info().intrinsic_matrix())
        U, V = np.meshgrid(np.arange(h), np.arange(w))
        points_in_camera_frame = np.vstack([
            U.flatten(),
            V.flatten(),
            np.ones(w*h)])
        points_in_camera_frame = Kinv.dot(points_in_camera_frame) * \
            depth_image.flatten()

        # The depth camera has some offset from the camera's root frame,
        # so take than into account.
        pose_mat = self.camera.depth_camera_optical_pose().matrix()
        points_in_camera_frame = pose_mat[0:3, 0:3].dot(points_in_camera_frame)
        points_in_camera_frame += np.tile(pose_mat[0:3, 3], [w*h, 1]).T

        kinsol = self.rbt.doKinematics(x[:self.rbt.get_num_positions()])
        points_in_world_frame = self.rbt.transformPoints(
            kinsol,
            points_in_camera_frame,
            self.camera.frame().get_frame_index(),
            0)

        # Color points according to their normalized height
        min_height = 0.0
        max_height = 2.0
        colors = cm.jet(
            (points_in_world_frame[2, :]-min_height)/(max_height-min_height)
            ).T[0:3, :]

        self.vis[self.prefix]["points"].set_object(
            g.PointCloud(position=points_in_world_frame,
                         color=colors,
                         size=0.005))
