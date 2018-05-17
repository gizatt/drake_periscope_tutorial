# -*- coding: utf8 -*-

import argparse
import math
import os.path
import time

import matplotlib.animation as animation
import numpy as np
import scipy as sp
import scipy.spatial

import pydrake
from pydrake.all import (
    Adder,
    AddFlatTerrainToWorld,
    AddModelInstancesFromSdfString,
    AddModelInstanceFromUrdfFile,
    AddModelInstanceFromUrdfStringSearchingInRosPackages,
    AffineSystem,
    CompliantContactModelParameters,
    CompliantMaterial,
    ConstantVectorSource,
    Context,
    DiagramBuilder,
    FloatingBaseType,
    Gain,
    LeafSystem,
    Multiplexer,
    PiecewisePolynomial,
    PortDataType,
    RigidBodyFrame,
    RigidBodyPlant,
    RigidBodyTree,
    Shape,
    SignalLogger,
    Simulator,
    UniformRandomSource
)
from pydrake.solvers import ik

from underactuated.utils import FindResource
from underactuated.meshcat_rigid_body_visualizer import (
    MeshcatRigidBodyVisualizer)


def setup_kuka(rbt):
    iiwa_urdf_path = os.path.join(
        pydrake.getDrakePath(),
        "manipulation", "models", "iiwa_description", "urdf",
        "iiwa14_primitive_collision.urdf")

    wsg50_sdf_path = os.path.join(
        pydrake.getDrakePath(),
        "manipulation", "models", "wsg_50_description", "sdf",
        "schunk_wsg_50_ball_contact.sdf")

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


def plan_grasping_trajectory(rbt, q0, target_ee_pose):
    nq = rbt.get_num_positions()
    q_des_full = np.zeros(nq)

    # Joints we want to search over
    ik_free_joint_names = [
        "iiwa_joint_1",
        "iiwa_joint_2",
        "iiwa_joint_3",
        "iiwa_joint_4",
        "iiwa_joint_5",
        "iiwa_joint_6",
        "iiwa_joint_7"
        ]
    # Extract these joint indices
    free_config_inds = []
    constrained_config_inds = []
    for i in range(rbt.get_num_bodies()):
        body = rbt.get_body(i)
        if body.has_joint():
            joint = body.getJoint()
            if joint.get_name() in ik_free_joint_names:
                free_config_inds += range(
                    body.get_position_start_index(),
                    body.get_position_start_index() +
                    joint.get_num_positions())
            else:
                constrained_config_inds += range(
                    body.get_position_start_index(),
                    body.get_position_start_index() +
                    joint.get_num_positions())

    # Assemble IK constraints
    constraints = []

    # Constrain the non-searched-over joints
    posture_constraint = ik.PostureConstraint(rbt)
    posture_constraint.setJointLimits(
        constrained_config_inds,
        q0[constrained_config_inds]-0.01, q0[constrained_config_inds]+0.01)
    constraints.append(posture_constraint)

    # Constrain the ee frame to lie on the target point
    # facing in the target orientation
    ee_frame = rbt.findFrame("iiwa_frame_ee").get_frame_index()
    constraints.append(
        ik.WorldPositionConstraint(
            rbt, ee_frame, np.zeros((3, 1)),
            target_ee_pose[0:3]-0.01, target_ee_pose[0:3]+0.01)
    )
    constraints.append(
        ik.WorldEulerConstraint(
            rbt, ee_frame,
            target_ee_pose[3:6]-0.01, target_ee_pose[3:6]+0.01)
    )

    options = ik.IKoptions(rbt)
    results = ik.InverseKin(
        rbt, q0, q0, constraints, options)
    print results.q_sol, "info %d" % results.info[0]
    return results.q_sol[0], results.info[0]

def render_system_with_graphviz(system, output_file):
    from graphviz import Source
    string = system.GetGraphvizString()
    src = Source(string)
    src.render(output_file, view=False)


if __name__ == "__main__":
    rbt = RigidBodyTree()
    setup_kuka(rbt)
    Tview = np.array([[1., 0., 0., 0.],
                      [0., 0., 1., 0.],
                      [0., 0., 0., 1.]],
                     dtype=np.float64)
    pbrv = MeshcatRigidBodyVisualizer(rbt, draw_timestep=0.001)

    nq = rbt.get_num_positions()
    timestep = 0.005

    q0, info = plan_grasping_trajectory(
        rbt,
        rbt.getZeroConfiguration(),
        np.array([0.75, 0., 0.95, -0.75, 0., -1.57]))

    pbrv.draw(q0)
    exit()
    rbplant = RigidBodyPlant(rbt)
    nx = rbplant.get_num_states()

    allmaterials = CompliantMaterial()
    allmaterials.set_youngs_modulus(1E8) # default 1E9
    allmaterials.set_dissipation(1.0) # default 0.32
    allmaterials.set_friction(1.0) # default 0.9.
    rbplant.set_default_compliant_material(allmaterials)

    params = CompliantContactModelParameters()
    params.v_stiction_tolerance = 0.01 # 0.01
    params.characteristic_radius = 0.0002 # 0.0002
    rbplant.set_contact_model_parameters(params)

    builder = DiagramBuilder()
    rbplant_sys = builder.AddSystem(rbplant)

    torque = 100.0
    print('Simulating with uniform random torque on [%f, %f]'
          % (-torque, torque))

    for i in range(rbplant_sys.get_num_input_ports()):
        input_port = rbplant_sys.get_input_port(i)
        N = input_port.size()
        random_source = builder.AddSystem(UniformRandomSource(
                                N, sampling_interval_sec=1.))
        noise_shifter = builder.AddSystem(AffineSystem(
            A=[0.],
            B=np.zeros((1, N)),
            f0=[0.],
            C=np.zeros((N, 1)),
            D=np.eye(N)*torque*2.,
            y0=-1.*np.ones((N, 1))*torque))
        builder.Connect(random_source.get_output_port(0),
                        noise_shifter.get_input_port(0))
        builder.Connect(noise_shifter.get_output_port(0),
                        input_port)

    # Visualize
    visualizer = builder.AddSystem(pbrv)
    output_sizes = [rbplant_sys.get_output_port(i).size()
                    for i in range(rbplant_sys.get_num_output_ports())]
    print output_sizes
    state_combiner = Multiplexer(output_sizes)
    builder.Connect(rbplant_sys.get_output_port(0),
                    visualizer.get_input_port(0))

    # And also log
    signalLogRate = 60
    signalLogger = builder.AddSystem(SignalLogger(nx))
    signalLogger._DeclarePeriodicPublish(1. / signalLogRate, 0.0)
    builder.Connect(rbplant_sys.get_output_port(0),
                    signalLogger.get_input_port(0))

    diagram = builder.Build()
    render_system_with_graphviz(diagram, "view.gv")

    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(True)

    # TODO(russt): Clean up state vector access below.
    state = simulator.get_mutable_context().get_mutable_state()\
                     .get_mutable_continuous_state().get_mutable_vector()

    initial_state = np.zeros(state.size())
    initial_state[0:q0.shape[0]] = q0
    state.SetFromVector(initial_state)

    print simulator.get_integrator().get_target_accuracy()
    #simulator.get_integrator().set_target_accuracy(0.05)
    simulator.get_integrator().set_fixed_step_mode(True)
    simulator.get_integrator().set_maximum_step_size(timestep)

    simulator.StepTo(10)
    print(state.CopyToVector())

    # Generate an animation of whatever happened
    ani = visualizer.animate(signalLogger)
