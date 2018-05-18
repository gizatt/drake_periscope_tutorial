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
    return free_config_inds, constrained_config_inds


class KukaController(LeafSystem):
    def __init__(self, rbt, plant, qtraj, ts,
                 control_period=0.0333,
                 print_period=0.01):
        LeafSystem.__init__(self)
        self.set_name("Kuka Controller")

        self.qtraj = PiecewisePolynomial.FirstOrderHold(ts, np.vstack(qtraj).T)

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

        self._DeclareInputPort(PortDataType.kVectorValued,
                               rbt.get_num_positions() +
                               rbt.get_num_velocities())

        self._DeclareDiscreteState(self.nu)
        self._DeclarePeriodicDiscreteUpdate(period_sec=control_period)
        self._DeclareVectorOutputPort(
            BasicVector(self.nu),
            self._DoCalcVectorOutput)

    ''' This is called on every discrete state update (at the
        specified control period), and expects the discrete state
        to be updated with the new discrete state after the update.

        For this system, the state is the output we'd like to produce
        (for the complete robot). This system could be created
        in a stateless way pretty easily -- through a combination of
        limiting the publish rate of the system, and adding multiplexers
        after the system to split the output into the
        individual-finger-sized chunks that the hand plant wants
        to consume. '''
    def _DoCalcDiscreteVariableUpdates(self, context, events, discrete_state):
        # Call base method to ensure we do not get recursion.
        LeafSystem._DoCalcDiscreteVariableUpdates(
            self, context, events, discrete_state)

        new_control_input = discrete_state. \
            get_mutable_vector().get_mutable_value()
        x = self.EvalVectorInput(context, 0).get_value()
        q = x[:self.nq]
        v = x[self.nq:]
        old_u = discrete_state.get_mutable_vector().get_mutable_value()

        target_q = self.qtraj.value(context.get_time())
        err = (target_q[self.controlled_inds, 0] - x[self.controlled_inds])

        kinsol = rbt.doKinematics(q, v)
        # Get the full LHS of the manipulator equations
        # given the current config and a desired qdd
        qdd_des = np.zeros(rbt.get_num_positions())
        qdd_des[self.controlled_inds] = 100.*err - 5.*v[self.controlled_inds]
        lhs = rbt.inverseDynamics(kinsol, external_wrenches={}, vd=qdd_des)
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
                 control_period=0.0333,
                 print_period=1.0):
        LeafSystem.__init__(self)
        self.set_name("Hand Controller")

        # Copy lots of stuff
        self.rbt = rbt
        self.plant = plant
        self.nu = plant.get_input_port(1).size()
        self.print_period = print_period
        self.last_print_time = -print_period
        self.shut_up = False

        self._DeclareInputPort(PortDataType.kVectorValued,
                               plant.get_output_port(1).size())

        self._DeclareDiscreteState(self.nu)
        self._DeclarePeriodicDiscreteUpdate(period_sec=control_period)
        self._DeclareVectorOutputPort(
            BasicVector(self.nu),
            self._DoCalcVectorOutput)

    ''' This is called on every discrete state update (at the
        specified control period), and expects the discrete state
        to be updated with the new discrete state after the update.
        
        For this system, the state is the output we'd like to produce
        (for the complete robot). This system could be created
        in a stateless way pretty easily -- through a combination of
        limiting the publish rate of the system, and adding multiplexers
        after the system to split the output into the
        individual-finger-sized chunks that the hand plant wants
        to consume. '''
    def _DoCalcDiscreteVariableUpdates(self, context, events, discrete_state):
        # Call base method to ensure we do not get recursion.
        LeafSystem._DoCalcDiscreteVariableUpdates(
            self, context, events, discrete_state)

        new_control_input = discrete_state. \
            get_mutable_vector().get_mutable_value()
        x = self.EvalVectorInput(context, 0).get_value()
        old_u = discrete_state.get_mutable_vector().get_mutable_value()
        new_u = 100.
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


def setup_kuka(rbt):
    iiwa_urdf_path = os.path.join(
        pydrake.getDrakePath(),
        "manipulation", "models", "iiwa_description", "urdf",
        "iiwa14_polytope_collision.urdf")

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


def plan_grasping_configuration(rbt, q0, target_ee_pose):
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

    return results.q_sol, ts, results.info[0]


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
    time.sleep(1.0)

    nq = rbt.get_num_positions()
    timestep = 0.005

    q0 = rbt.getZeroConfiguration()
    qtraj, ts, info = plan_grasping_trajectory(
        rbt,
        q0,
        np.array([0.75, 0., 0.95, -0.75, 0., -1.57]),
        10, 1.0)

    # dt = 3. / 100.
    # time.sleep(dt)
    # for k in range(10):
    #    for qi in qtraj:
    #        pbrv.draw(qi)
    #        time.sleep(dt)
    # exit()

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

    '''
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
    '''

    kuka_controller = builder.AddSystem(
        KukaController(rbt, rbplant_sys, qtraj, ts))
    builder.Connect(kuka_controller.get_output_port(0),
                    rbplant_sys.get_input_port(0))
    builder.Connect(rbplant_sys.state_output_port(),
                    kuka_controller.get_input_port(0))

    hand_controller = builder.AddSystem(HandController(rbt, rbplant_sys))
    builder.Connect(hand_controller.get_output_port(0),
                    rbplant_sys.get_input_port(1))
    builder.Connect(rbplant_sys.get_output_port(1),
                    hand_controller.get_input_port(0))

    # Visualize
    visualizer = builder.AddSystem(pbrv)
    builder.Connect(rbplant_sys.state_output_port(),
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
    simulator.get_integrator().set_target_accuracy(0.05)
    #simulator.get_integrator().set_fixed_step_mode(True)
    #simulator.get_integrator().set_maximum_step_size(timestep)

    simulator.StepTo(10)
    print(state.CopyToVector())

    # Generate an animation of whatever happened
    ani = visualizer.animate(signalLogger)
