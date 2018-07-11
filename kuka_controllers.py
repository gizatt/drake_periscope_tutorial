# -*- coding: utf8 -*-

import numpy as np
import pydrake
from pydrake.all import (
    BasicVector,
    LeafSystem,
    PortDataType,
)
import kuka_utils


class KukaController(LeafSystem):
    def __init__(self, rbt, plant,
                 control_period=0.005,
                 print_period=0.5):
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

        self.controlled_inds, _ = kuka_utils.extract_position_indices(
            rbt, self.controlled_joint_names)
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

        kinsol = self.rbt.doKinematics(q, v)
        # Get the full LHS of the manipulator equations
        # given the current config and desired accelerations
        vd_des = np.zeros(self.rbt.get_num_positions())
        vd_des[self.controlled_inds] = 1000.*qerr + 100*verr
        lhs = self.rbt.inverseDynamics(kinsol, external_wrenches={}, vd=vd_des)
        new_u = self.B_inv.dot(lhs[self.controlled_inds])
        new_control_input[:] = new_u

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
                 control_period=0.001):
        LeafSystem.__init__(self)
        self.set_name("Hand Controller")

        self.controlled_joint_names = [
            "left_finger_sliding_joint",
            "right_finger_sliding_joint"
        ]

        self.max_force = 100.  # gripper max closing / opening force

        self.controlled_inds, _ = kuka_utils.extract_position_indices(
            rbt, self.controlled_joint_names)

        self.nu = plant.get_input_port(1).size()
        self.nq = rbt.get_num_positions()

        self.robot_state_input_port = \
            self._DeclareInputPort(PortDataType.kVectorValued,
                                   rbt.get_num_positions() +
                                   rbt.get_num_velocities())

        self.setpoint_input_port = \
            self._DeclareInputPort(PortDataType.kVectorValued,
                                   1)

        self._DeclareDiscreteState(self.nu)
        self._DeclarePeriodicDiscreteUpdate(period_sec=control_period)
        self._DeclareVectorOutputPort(
            BasicVector(self.nu),
            self._DoCalcVectorOutput)

    def _DoCalcDiscreteVariableUpdates(self, context, events, discrete_state):
        # Call base method to ensure we do not get recursion.
        # (This makes sure relevant event handlers get called.)
        LeafSystem._DoCalcDiscreteVariableUpdates(
            self, context, events, discrete_state)

        new_control_input = discrete_state. \
            get_mutable_vector().get_mutable_value()
        x = self.EvalVectorInput(
            context, self.robot_state_input_port.get_index()).get_value()

        gripper_width_des = self.EvalVectorInput(
            context, self.setpoint_input_port.get_index()).get_value()

        q_full = x[:self.nq]
        v_full = x[self.nq:]

        q = q_full[self.controlled_inds]
        q_des = np.array([-gripper_width_des[0], gripper_width_des[0]])
        v = v_full[self.controlled_inds]
        v_des = np.zeros(2)

        qerr = q_des - q
        verr = v_des - v

        Kp = 1000.
        Kv = 100.
        new_control_input[:] = np.clip(
            Kp * qerr + Kv * verr, -self.max_force, self.max_force)

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
        if context.get_time() > self.qtraj.end_time():
            new_state[:] = 0.
        else:
            new_state[:] = 0.1

    def _DoCalcKukaSetpointOutput(self, context, y_data):

        t = context.get_time()

        t_end = self.qtraj.end_time()
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
        nq_plan = target_q.shape[0]
        kuka_setpoint[:nq_plan] = target_q[:, 0]
        kuka_setpoint[self.nq:(self.nq+nq_plan)] = target_v[:, 0]

    def _DoCalcHandSetpointOutput(self, context, y_data):
        state = context.get_discrete_state_vector().get_value()
        y = y_data.get_mutable_value()
        # Get the ith finger control output
        y[:] = state[0]