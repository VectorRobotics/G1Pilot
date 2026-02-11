
import time
import mujoco
import mujoco.viewer
from threading import Thread, Event
import threading
import rclpy # Import rclpy
from rclpy.node import Node # Import Node
from sensor_msgs.msg import JointState # Import JointState message type

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_bridge import UnitreeSdk2Bridge, ElasticBand
# from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_

import sys

import config

import ipdb

# stop_event = Event()

locker = threading.Lock()

mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
mj_data = mujoco.MjData(mj_model)

if config.ENABLE_ELASTIC_BAND:
    elastic_band = ElasticBand()
    band_attached_link = mj_model.body("torso_link").id
    viewer = mujoco.viewer.launch_passive(
        mj_model, mj_data, key_callback=elastic_band.MujuocoKeyCallback
    )
else:
    viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

mj_model.opt.timestep = config.SIMULATE_DT
num_motor_ = mj_model.nu # overall DOF, should = 29 for g1
dim_motor_sensor_ = 3 * num_motor_ # pos, vel, torque for each DOF

# Global variable to store the latest joint states
exp_joint_states = None
ros_node = None



time.sleep(0.2)

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('mujoco_joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'ik/joint_states', # Topic name
            self.listener_callback,
            10)
        self.get_logger().info("Mujoco JointState Subscriber Node has started.")

    def listener_callback(self, msg: JointState):
        global exp_joint_states
        locker.acquire() # Acquire lock before updating global variables
        exp_joint_states = msg
        locker.release() # Release lock
        # self.get_logger().info(f"Received joint states for: {msg.name}") # Optional: log received messages


def SimulationThread():
    global mj_data, mj_model, exp_joint_states

    node = JointStateSubscriber()
    # Create a separate thread for spinning the ROS 2 node
    ros_spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_spin_thread.start()


    ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
    unitree = UnitreeSdk2Bridge(mj_model, mj_data)

    if config.USE_JOYSTICK:
        unitree.SetupJoystick(device_id=0, js_type=config.JOYSTICK_TYPE)
    if config.PRINT_SCENE_INFORMATION:
        unitree.PrintSceneInformation()

    # while viewer.is_running() and not stop_event.is_set():
    while viewer.is_running():
        step_start = time.perf_counter()

        locker.acquire()

        if config.ENABLE_ELASTIC_BAND:
            if elastic_band.enable:
                mj_data.xfrc_applied[band_attached_link, :3] = elastic_band.Advance(
                    mj_data.qpos[:3], mj_data.qvel[:3]
                )
        
        unitree.LowCmdHandler_Joint(exp_joint_states)

        mujoco.mj_step(mj_model, mj_data)

        locker.release()

        time_until_next_step = mj_model.opt.timestep - (
            time.perf_counter() - step_start
        )
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
            
    # Shutdown ROS 2 for this node when the simulation ends
    ros_spin_thread.join()
    node.destroy_node()
    print("SimulationThread: Stopping ROS 2 node and spin thread...")
    

def PhysicsViewerThread():
    global viewer
    # while viewer.is_running() and not stop_event.is_set():
    while viewer.is_running():
        locker.acquire() # thread lock, mutex
        viewer.sync()
        locker.release()
        time.sleep(config.VIEWER_DT)


if __name__ == "__main__":
    # Initialize ROS 2 once in the main thread
    rclpy.init(args=None)

    viewer_thread = Thread(target=PhysicsViewerThread)
    sim_thread = Thread(target=SimulationThread)

    try:
        viewer_thread.start()
        sim_thread.start()
        
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Signaling threads to stop...")
        # stop_event.set() 

    finally:
        # stop_event.set()
        viewer_thread.join()
        sim_thread.join()

        rclpy.shutdown()

        if viewer.is_running():
            viewer.close()

        print("\nAll threads stopped and resources cleaned up.")
