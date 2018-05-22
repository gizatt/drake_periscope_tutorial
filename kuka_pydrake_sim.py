# -*- coding: utf8 -*-

import argparse
import math
import os.path
import time

import matplotlib.animation as animation
import matplotlib.pyplot as plt
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
    BasicVector,
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
    RungeKutta2Integrator,
    Shape,
    SignalLogger,
    Simulator,
    UniformRandomSource,
)
from pydrake.solvers import ik

from underactuated.utils import FindResource
from underactuated.meshcat_rigid_body_visualizer import (
    MeshcatRigidBodyVisualizer)


def extract_position_indices(rbt, controlled_joint_names):
    # Joints we want to search over
    # Extract these joint indices
    free_config_inds = []
    constrained_config_inds = []
    for i in range(rbt.get_num_bodies()):
        body = rbt.get_body(i)
        if body.has_joint():
            joint = body.getJoint()
            if joint.get_name() in controlled_joint_names:
                free_config_inds += range(
                    body.get_position_start_index(),
                    body.get_position_start_index() +
                    joint.get_num_positions())
            else:
                constrained_config_inds += range(
                    body.get_position_start_index(),
                    body.get_position_start_index() +
                    joint.get_num_positions())
    if len(controlled_joint_names) != len(free_config_inds):
        print("Didn't find all requested controlled joint names.")
    return free_config_inds, constrained_config_inds


class KukaController(LeafSystem):
    def __init__(self, rbt, plant,
                 control_period=0.005,
                 print_period=0.01):
        LeafSystem.__init__(self)
        self.set_name("Kuka Controller")

        self.controlled_joint_names = [
            "iiwa_joint_1",
            "iiwa_joint_2",
            "iiwa_joint_3",
            "iiwa_joint_4",
            "iiwa_joint_5",
            "iiwa_joint_6",
            "iiwa_joint_7"
        ]

        self.controlled_inds, _ = \
            extract_position_indices(rbt, self.controlled_joint_names)
        # Extract the full-rank bit of B, and verify that it's full rank
        self.nq_reduced = len(self.controlled_inds)
        self.B = np.empty((self.nq_reduced, self.nq_reduced))
        for k in range(self.nq_reduced):
            for l in range(self.nq_reduced):
                self.B[k, l] = rbt.B[self.controlled_inds[k],
                                     self.controlled_inds[l]]
        if np.linalg.matrix_rank(self.B) < self.nq_reduced:
            print "The joint set specified is underactuated."
            sys.exit(-1)
        self.B_inv = np.linalg.inv(self.B)
        # Copy lots of stuff
        self.rbt = rbt
        self.nq = rbt.get_num_positions()
        self.plant = plant
        self.nu = plant.get_input_port(0).size()
        self.print_period = print_period
        self.last_print_time = -print_period
        self.shut_up = False

        self.robot_state_input_port = \
            self._DeclareInputPort(PortDataType.kVectorValued,
                                   rbt.get_num_positions() +
                                   rbt.get_num_velocities())

        self.setpoint_input_port = \
            self._DeclareInputPort(PortDataType.kVectorValued,
                                   rbt.get_num_positions() +
                                   rbt.get_num_velocities())

        self._DeclareDiscreteState(self.nu)
        self._DeclarePeriodicDiscreteUpdate(period_sec=control_period)
        self._DeclareVectorOutputPort(
            BasicVector(self.nu),
            self._DoCalcVectorOutput)
        self._DeclarePeriodicPublish(0.001, 0.0)

    def _DoCalcDiscreteVariableUpdates(self, context, events, discrete_state):
        # Call base method to ensure we do not get recursion.
        # (This makes sure relevant event handlers get called.)
        LeafSystem._DoCalcDiscreteVariableUpdates(
            self, context, events, discrete_state)

        new_control_input = discrete_state. \
            get_mutable_vector().get_mutable_value()
        x = self.EvalVectorInput(
            context, self.robot_state_input_port.get_index()).get_value()
        x_des = self.EvalVectorInput(
            context, self.setpoint_input_port.get_index()).get_value()
        q = x[:self.nq]
        v = x[self.nq:]
        q_des = x_des[:self.nq]
        v_des = x_des[self.nq:]

        qerr = (q_des[self.controlled_inds] - q[self.controlled_inds])
        verr = (v_des[self.controlled_inds] - v[self.controlled_inds])

        kinsol = rbt.doKinematics(q, v)
        # Get the full LHS of the manipulator equations
        # given the current config and desired accelerations
        vd_des = np.zeros(rbt.get_num_positions())
        vd_des[self.controlled_inds] = 1000.*qerr + 100*verr
        lhs = rbt.inverseDynamics(kinsol, external_wrenches={}, vd=vd_des)
        new_u = self.B_inv.dot(lhs[self.controlled_inds])
        new_control_input[:] = new_u

    ''' This is called whenever this system needs to publish
        output. We did some magic in the constructor to add
        an extra argument to tell the function what finger's
        control input to return. It looks up into the
        current state what the current complete output is,
        and returns the torques for only finger i.'''
    def _DoCalcVectorOutput(self, context, y_data):
        if (self.print_period and
                context.get_time() - self.last_print_time
                >= self.print_period):
            print "t: ", context.get_time()
            self.last_print_time = context.get_time()
        control_output = context.get_discrete_state_vector().get_value()
        y = y_data.get_mutable_value()
        # Get the ith finger control output
        y[:] = control_output[:]


class HandController(LeafSystem):
    def __init__(self, rbt, plant,
                 control_period=0.0333):
        LeafSystem.__init__(self)
        self.set_name("Hand Controller")

        # Copy lots of stuff

        self.rbt = rbt
        self.plant = plant
        self.nu = plant.get_input_port(1).size()

        '''
        self.controlled_joint_names = [
            "left_finger_sliding_joint",
            "right_finger_sliding_joint"
        ]
        self.controlled_inds, _ = \
            extract_position_indices(rbt, self.controlled_joint_names)
        '''

        # Input: scalar desired grasp force
        self.setpoint_input_port = \
            self._DeclareInputPort(PortDataType.kVectorValued, 1)

        self._DeclareDiscreteState(self.nu)
        self._DeclarePeriodicDiscreteUpdate(period_sec=control_period)
        self._DeclareVectorOutputPort(
            BasicVector(self.nu),
            self._DoCalcVectorOutput)

    def _DoCalcDiscreteVariableUpdates(self, context, events, discrete_state):
        # Call base method to ensure we do not get recursion.
        LeafSystem._DoCalcDiscreteVariableUpdates(
            self, context, events, discrete_state)

        new_control_input = discrete_state. \
            get_mutable_vector().get_mutable_value()
        desired_force = self.EvalVectorInput(context, 0).get_value()[0]
        new_control_input[:] = [desired_force]

    ''' This is called whenever this system needs to publish
        output. We did some magic in the constructor to add
        an extra argument to tell the function what finger's
        control input to return. It looks up into the
        current state what the current complete output is,
        and returns the torques for only finger i.'''
    def _DoCalcVectorOutput(self, context, y_data):
        control_output = context.get_discrete_state_vector().get_value()
        y = y_data.get_mutable_value()
        # Get the ith finger control output
        y[:] = control_output[:]


class ManipStateMachine(LeafSystem):
    ''' Encodes the high-level logic
        for the manipulation system.

        This implementation is fairly minimal.
        It is supplied with an open-loop
        trajectory (presumably, to grasp the object from a
        known position). At runtime, it spools
        out pose goals for the robot according to
        this trajectory. Once the trajectory has been
        executed, it closes the gripper, waits
        a second, and then plays the trajectory back in reverse
        to bring the robot back to its original posture.
    '''
    def __init__(self, rbt, plant, qtraj):
        LeafSystem.__init__(self)
        self.set_name("Manipulation State Machine")

        self.qtraj = qtraj

        self.gripper_closing_force = 10.
        self.gripper_opening_force = -10.

        self.rbt = rbt
        self.nq = rbt.get_num_positions()
        self.plant = plant

        self.robot_state_input_port = \
            self._DeclareInputPort(PortDataType.kVectorValued,
                                   rbt.get_num_positions() +
                                   rbt.get_num_velocities())

        self._DeclareDiscreteState(1)
        self._DeclarePeriodicDiscreteUpdate(period_sec=0.001)

        self.kuka_setpoint_output_port = \
            self._DeclareVectorOutputPort(
                BasicVector(rbt.get_num_positions() +
                            rbt.get_num_velocities()),
                self._DoCalcKukaSetpointOutput)
        self.hand_setpoint_output_port = \
            self._DeclareVectorOutputPort(BasicVector(1),
                                          self._DoCalcHandSetpointOutput)

        self._DeclarePeriodicPublish(0.01, 0.0)

    def _DoCalcDiscreteVariableUpdates(self, context, events, discrete_state):
        # Call base method to ensure we do not get recursion.
        LeafSystem._DoCalcDiscreteVariableUpdates(
            self, context, events, discrete_state)

        new_state = discrete_state. \
            get_mutable_vector().get_mutable_value()
        # Close gripper after plan has been executed
        if context.get_time() > qtraj.end_time():
            new_state[:] = self.gripper_closing_force
        else:
            new_state[:] = self.gripper_opening_force

    def _DoCalcKukaSetpointOutput(self, context, y_data):

        t = context.get_time()

        t_end = qtraj.end_time()
        if t < t_end:
            virtual_time = t
        else:
            virtual_time = t_end - (t - t_end)

        dt = 0.01  # Look-ahead for estimating target velocity

        target_q = self.qtraj.value(virtual_time)
        target_qn = self.qtraj.value(virtual_time+dt)
        # This is pretty inefficient and inaccurate -- TODO(gizatt)
        # velocity target directly from the trajectory object somehow.
        target_v = (target_qn - target_q) / dt
        if t >= t_end:
            target_v *= -1.
        kuka_setpoint = y_data.get_mutable_value()
        # Get the ith finger control output
        kuka_setpoint[:self.nq] = target_q[:, 0]
        kuka_setpoint[self.nq:] = target_v[:, 0]

    def _DoCalcHandSetpointOutput(self, context, y_data):
        state = context.get_discrete_state_vector().get_value()
        y = y_data.get_mutable_value()
        # Get the ith finger control output
        y[:] = state[0]


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


def plan_grasping_configuration(rbt, q0, target_ee_pose):
    ''' Performs IK for a single point in time
        to get the Kuka's gripper to a specified
        pose in space. '''
    nq = rbt.get_num_positions()
    q_des_full = np.zeros(nq)

    controlled_joint_names = [
        "iiwa_joint_1",
        "iiwa_joint_2",
        "iiwa_joint_3",
        "iiwa_joint_4",
        "iiwa_joint_5",
        "iiwa_joint_6",
        "iiwa_joint_7"
        ]
    free_config_inds, constrained_config_inds = \
        extract_position_indices(rbt, controlled_joint_names)

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


def plan_grasping_trajectory(rbt, q0, target_ee_pose, n_pts, end_time):
    ''' Solves IK at a series of sample times (connected with a
    spline) to generate a trajectory to bring the Kuka from an
    initial pose q0 to a final end effector pose in the specified
    time, using the specified number of knot points. '''
    nq = rbt.get_num_positions()
    q_des_full = np.zeros(nq)

    # Create knot points
    ts = np.linspace(0., end_time, n_pts)

    controlled_joint_names = [
        "iiwa_joint_1",
        "iiwa_joint_2",
        "iiwa_joint_3",
        "iiwa_joint_4",
        "iiwa_joint_5",
        "iiwa_joint_6",
        "iiwa_joint_7"
        ]
    free_config_inds, constrained_config_inds = \
        extract_position_indices(rbt, controlled_joint_names)

    # Assemble IK constraints
    constraints = []

    # Constrain the non-searched-over joints for all time
    final_tspan = np.array([end_time, end_time])
    posture_constraint = ik.PostureConstraint(rbt, final_tspan)
    posture_constraint.setJointLimits(
        constrained_config_inds,
        q0[constrained_config_inds]-0.01, q0[constrained_config_inds]+0.01)
    constraints.append(posture_constraint)

    # Constrain all joints to be the initial posture at the start time
    start_tspan = np.array([0., 0.])
    posture_constraint = ik.PostureConstraint(rbt, start_tspan)
    posture_constraint.setJointLimits(
        free_config_inds,
        q0[free_config_inds]-0.01, q0[free_config_inds]+0.01)
    constraints.append(posture_constraint)

    # Constrain the ee frame to lie on the target point
    # facing in the target orientation at the final time
    ee_frame = rbt.findFrame("iiwa_frame_ee").get_frame_index()
    constraints.append(
        ik.WorldPositionConstraint(
            rbt, ee_frame, np.zeros((3, 1)),
            target_ee_pose[0:3]-0.01, target_ee_pose[0:3]+0.01,
            tspan=final_tspan)
    )
    constraints.append(
        ik.WorldEulerConstraint(
            rbt, ee_frame,
            target_ee_pose[3:6]-0.01, target_ee_pose[3:6]+0.01,
            tspan=final_tspan)
    )

    # Seed and nom are both the initial repeated for the #
    # of knot points
    q_seed = np.tile(q0, [1, n_pts])
    q_nom = np.tile(q0, [1, n_pts])
    options = ik.IKoptions(rbt)
    results = ik.InverseKinTraj(rbt, ts, q_seed, q_nom,
                                constraints, options)

    qtraj = PiecewisePolynomial.Pchip(ts, np.vstack(results.q_sol).T, True)

    return qtraj, results.info[0]


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
    pbrv = MeshcatRigidBodyVisualizer(rbt, draw_timestep=0.01)
    time.sleep(2.0)

    nq = rbt.get_num_positions()
    timestep = 0.0002

    q0 = rbt.getZeroConfiguration()
    qtraj, info = plan_grasping_trajectory(
        rbt,
        q0,
        np.array([0.75, 0., 0.95, -0.75, 0., -1.57]),
        10, 2.0)

    rbplant = RigidBodyPlant(rbt)
    rbplant.set_name("Rigid Body Plant")
    nx = rbplant.get_num_states()

    allmaterials = CompliantMaterial()
    allmaterials.set_youngs_modulus(1E9) # default 1E9
    allmaterials.set_dissipation(0.32) # default 0.32
    allmaterials.set_friction(0.9) # default 0.9.
    rbplant.set_default_compliant_material(allmaterials)

    params = CompliantContactModelParameters()
    params.v_stiction_tolerance = 0.01 # 0.01
    params.characteristic_radius = 0.0002 # 0.0002
    rbplant.set_contact_model_parameters(params)

    builder = DiagramBuilder()
    rbplant_sys = builder.AddSystem(rbplant)

    manip_state_machine = builder.AddSystem(
        ManipStateMachine(rbt, rbplant_sys, qtraj))
    builder.Connect(rbplant_sys.state_output_port(),
                    manip_state_machine.robot_state_input_port)

    kuka_controller = builder.AddSystem(
        KukaController(rbt, rbplant_sys))
    builder.Connect(rbplant_sys.state_output_port(),
                    kuka_controller.robot_state_input_port)
    builder.Connect(manip_state_machine.kuka_setpoint_output_port,
                    kuka_controller.setpoint_input_port)
    builder.Connect(kuka_controller.get_output_port(0),
                    rbplant_sys.get_input_port(0))

    hand_controller = builder.AddSystem(HandController(rbt, rbplant_sys))
    builder.Connect(manip_state_machine.hand_setpoint_output_port,
                    hand_controller.setpoint_input_port)
    builder.Connect(hand_controller.get_output_port(0),
                    rbplant_sys.get_input_port(1))

    # Visualize
    visualizer = builder.AddSystem(pbrv)
    builder.Connect(rbplant_sys.state_output_port(),
                    visualizer.get_input_port(0))

    # And also log state and setpoint
    def log_output(output_port, rate):
        logger = builder.AddSystem(SignalLogger(output_port.size()))
        logger._DeclarePeriodicPublish(1. / rate, 0.0)
        builder.Connect(output_port, logger.get_input_port(0))
        return logger
    state_log = log_output(rbplant_sys.get_output_port(0), 60.)
    setpoint_log = log_output(
        manip_state_machine.kuka_setpoint_output_port, 60.)
    kuka_control_log = log_output(
        kuka_controller.get_output_port(0), 60.)

    diagram = builder.Build()
    render_system_with_graphviz(diagram, "view.gv")

    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)

    # TODO(russt): Clean up state vector access below.
    state = simulator.get_mutable_context().get_mutable_state()\
                     .get_mutable_continuous_state().get_mutable_vector()

    initial_state = np.zeros(state.size())
    initial_state[0:q0.shape[0]] = q0
    state.SetFromVector(initial_state)

    # From iiwa_wsg_simulation.cc:
    # When using the default RK3 integrator, the simulation stops
    # advancing once the gripper grasps the box.  Grasping makes the
    # problem computationally stiff, which brings the default RK3
    # integrator to its knees.
    simulator.reset_integrator(
        RungeKutta2Integrator(diagram, timestep,
                              simulator.get_mutable_context()))

    simulator.StepTo(4.)
    print(state.CopyToVector())

    print state_log.data(), setpoint_log.data()
    plt.figure()
    plt.subplot(3, 1, 1)
    dims_to_draw = range(7)
    color = iter(plt.cm.rainbow(np.linspace(0, 1, 7)))
    for i in dims_to_draw:
        colorthis = next(color)
        plt.plot(state_log.sample_times(),
                 state_log.data()[i, :],
                 color=colorthis,
                 linestyle='solid',
                 label="q[%d]" % i)
        plt.plot(setpoint_log.sample_times(),
                 setpoint_log.data()[i, :],
                 color=colorthis,
                 linestyle='dashed',
                 label="q_des[%d]" % i)
    plt.ylabel("m")
    plt.grid(True)
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)

    plt.subplot(3, 1, 2)
    color = iter(plt.cm.rainbow(np.linspace(0, 1, 7)))
    for i in dims_to_draw:
        colorthis = next(color)
        plt.plot(state_log.sample_times(),
                 state_log.data()[nq + i, :],
                 color=colorthis,
                 linestyle='solid',
                 label="v[%d]" % i)
        plt.plot(setpoint_log.sample_times(),
                 setpoint_log.data()[nq + i, :],
                 color=colorthis,
                 linestyle='dashed',
                 label="v_des[%d]" % i)
    plt.ylabel("m/s")
    plt.grid(True)
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)

    plt.subplot(3, 1, 3)
    color = iter(plt.cm.rainbow(np.linspace(0, 1, 7)))
    for i in dims_to_draw:
        colorthis = next(color)
        plt.plot(kuka_control_log.sample_times(),
                 kuka_control_log.data()[i, :],
                 color=colorthis,
                 linestyle=':',
                 label="u[%d]" % i)
    plt.xlabel("t")
    plt.ylabel("N/m")
    plt.grid(True)
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.show()
