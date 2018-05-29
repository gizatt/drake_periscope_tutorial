# -*- coding: utf8 -*-

import argparse
import time

import matplotlib.pyplot as plt
import numpy as np

import pydrake
from pydrake.all import (
    DiagramBuilder,
    RgbdCamera,
    RigidBodyFrame,
    RigidBodyPlant,
    RigidBodyTree,
    RungeKutta2Integrator,
    Shape,
    SignalLogger,
    Simulator,
)

from underactuated.meshcat_rigid_body_visualizer import (
    MeshcatRigidBodyVisualizer)

import kuka_controllers
import kuka_ik
import kuka_utils

if __name__ == "__main__":
    # Construct the robot and its environment
    rbt = RigidBodyTree()
    kuka_utils.setup_kuka(rbt)

    # Set up a visualizer for the robot
    pbrv = MeshcatRigidBodyVisualizer(rbt, draw_timestep=0.01)
    # (wait while the visualizer warms up and loads in the models)
    time.sleep(2.0)

    # Plan a robot motion to maneuver from the initial posture
    # to a posture that we know should grab the object.
    # (Grasp planning is left as an exercise :))
    q0 = rbt.getZeroConfiguration()
    qtraj, info = kuka_ik.plan_grasping_trajectory(
        rbt,
        q0=q0,
        target_reach_pose=np.array([0.6, 0., 1.0, -0.75, 0., -1.57]),
        target_grasp_pose=np.array([0.8, 0., 0.9, -0.75, 0., -1.57]),
        n_knots=20,
        reach_time=1.5,
        grasp_time=2.0)

    # Make our RBT into a plant for simulation
    rbplant = RigidBodyPlant(rbt)
    rbplant.set_name("Rigid Body Plant")

    # Build up our simulation by spawning controllers and loggers
    # and connecting them to our plant.
    builder = DiagramBuilder()
    # The diagram takes ownership of all systems
    # placed into it.
    rbplant_sys = builder.AddSystem(rbplant)

    # Create a high-level state machine to guide the robot
    # motion...
    manip_state_machine = builder.AddSystem(
        kuka_controllers.ManipStateMachine(rbt, rbplant_sys, qtraj))
    builder.Connect(rbplant_sys.state_output_port(),
                    manip_state_machine.robot_state_input_port)

    # And spawn the controller that drives the Kuka to its
    # desired posture.
    kuka_controller = builder.AddSystem(
        kuka_controllers.KukaController(rbt, rbplant_sys))
    builder.Connect(rbplant_sys.state_output_port(),
                    kuka_controller.robot_state_input_port)
    builder.Connect(manip_state_machine.kuka_setpoint_output_port,
                    kuka_controller.setpoint_input_port)
    builder.Connect(kuka_controller.get_output_port(0),
                    rbplant_sys.get_input_port(0))

    # We could have a whole controller dedicated to the
    # hand, but the simplest controller is to feed the
    # force setpoint directly to the hand, since the gripper
    # we're using conveniently only has one input.
    builder.Connect(manip_state_machine.hand_setpoint_output_port,
                    rbplant_sys.get_input_port(1))

    # Hook up the visualizer we created earlier.
    visualizer = builder.AddSystem(pbrv)
    builder.Connect(rbplant_sys.state_output_port(),
                    visualizer.get_input_port(0))

    # Add a camera, too, though no controller or estimator
    # will consume the output of it.
    # - Add frame for camera fixture.
    camera_frame = RigidBodyFrame(
        name="rgbd camera frame", body=rbt.world(),
        xyz=[2, 0., 1.5], rpy=[-np.pi/4, 0., -np.pi])
    rbt.addFrame(camera_frame)
    camera = builder.AddSystem(
        RgbdCamera(name="camera", tree=rbt, frame=camera_frame,
                   z_near=0.5, z_far=2.0, fov_y=np.pi / 4,
                   width=320, height=240,
                   show_window=False))
    builder.Connect(rbplant_sys.state_output_port(),
                    camera.get_input_port(0))

    camera_meshcat_visualizer = builder.AddSystem(
        kuka_utils.RgbdCameraMeshcatVisualizer(camera, rbt))
    builder.Connect(camera.depth_image_output_port(),
                    camera_meshcat_visualizer.camera_input_port)
    builder.Connect(rbplant_sys.state_output_port(),
                    camera_meshcat_visualizer.state_input_port)

    # Hook up loggers for the robot state, the robot setpoints,
    # and the torque inputs.
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

    # Done! Compile it all together and visualize it.
    diagram = builder.Build()
    kuka_utils.render_system_with_graphviz(diagram, "view.gv")

    # Create a simulator for it.
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    # Simulator time steps will be very small, so don't
    # force the rest of the system to update every single time.
    simulator.set_publish_every_time_step(False)

    # The simulator simulates forward from a given Context,
    # so we adjust the simulator's initial Context to set up
    # the initial state.
    state = simulator.get_mutable_context().\
        get_mutable_continuous_state_vector()
    initial_state = np.zeros(state.size())
    initial_state[0:q0.shape[0]] = q0
    state.SetFromVector(initial_state)

    # From iiwa_wsg_simulation.cc:
    # When using the default RK3 integrator, the simulation stops
    # advancing once the gripper grasps the box.  Grasping makes the
    # problem computationally stiff, which brings the default RK3
    # integrator to its knees.
    timestep = 0.0002
    simulator.reset_integrator(
        RungeKutta2Integrator(diagram, timestep,
                              simulator.get_mutable_context()))

    # This kicks off simulation. Most of the run time will be spent
    # in this call.
    simulator.StepTo(4.)
    print("Final state: ", state.CopyToVector())

    # Do some plotting to show off accessing signal logger data.
    nq = rbt.get_num_positions()
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
