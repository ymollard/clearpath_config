# Software License Agreement (BSD)
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
# @copyright (c) 2024, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from typing import List
from clearpath_config.common.types.accessory import Accessory
from clearpath_config.common.types.ip import IP
from clearpath_config.common.types.port import Port
from clearpath_config.manipulators.types.grippers import Gripper
from clearpath_config.manipulators.types.manipulator import BaseManipulator


class BaseArm(BaseManipulator):
    MANIPULATOR_MODEL = "base"
    MANIPULATOR_TYPE = "arm"

    IP_ADDRESS = "ip"
    IP_PORT = "port"
    DEFAULT_IP_ADDRESS = "192.168.131.40"
    DEFAULT_IP_PORT = 10000

    URDF_PARAMETERS = {}

    def __init__(
            self,
            idx: int = None,
            name: str = None,
            ip: str = DEFAULT_IP_ADDRESS,
            port: int = DEFAULT_IP_PORT,
            ros_parameters: dict = BaseManipulator.ROS_PARAMETERS,
            ros_parameters_template: dict = BaseManipulator.ROS_PARAMETERS_TEMPLATE,
            parent: str = Accessory.PARENT,
            xyz: List[float] = Accessory.XYZ,
            rpy: List[float] = Accessory.RPY
            ) -> None:
        super().__init__(
            idx, name, ros_parameters, ros_parameters_template, parent, xyz, rpy)
        self.config = {}
        self.gripper = None
        # IP Address
        self.ip = IP(ip)
        # IP Port
        self.port = Port(port)
        # URDF Parameters
        self.urdf_parameters = dict(self.URDF_PARAMETERS)

    @classmethod
    def get_ip_from_idx(cls, idx: int) -> str:
        ip = cls.DEFAULT_IP_ADDRESS.split('.')
        network_id = ip[0:3]
        host_id = int(ip[-1]) + idx
        return '.'.join(network_id) + '.' + str(host_id)

    def set_idx(self, idx: int) -> None:
        super().set_idx(idx)
        if self.IP_ADDRESS not in self.config:
            self.ip = self.get_ip_from_idx(idx)
        if self.gripper:
            self.gripper.name = self.name + '_gripper'
            self.gripper.parent = self.name + '_end_effector_link'

    @property
    def ip(self) -> str:
        return str(self._ip)

    @ip.setter
    def ip(self, ip: str) -> None:
        self._ip = IP(str(ip))

    @property
    def port(self) -> int:
        return int(self._port)

    @port.setter
    def port(self, port: int) -> None:
        self._port = Port(int(port))

    def to_dict(self) -> dict:
        d = super().to_dict()
        d[self.IP_ADDRESS] = self.ip
        d[self.IP_PORT] = self.port
        if self.gripper:
            d['gripper'] = self.gripper.to_dict()
        else:
            d['gripper'] = None
        for k, v in self.urdf_parameters.items():
            if v:
                d[k] = v
        return d

    def from_dict(self, d: dict) -> None:
        self.config = d
        super().from_dict(d)
        if 'gripper' in d:
            self.gripper = Gripper(d['gripper']['model'])
            self.gripper.from_dict(d['gripper'])
            self.gripper.set_name('%s_gripper' % self.get_name())
            if 'parent' not in d['gripper']:
                self.gripper.set_parent('%s_end_effector_link' % self.get_name())
        if self.IP_ADDRESS in d:
            self.ip = d[self.IP_ADDRESS]
        if self.IP_PORT in d:
            self.port = d[self.IP_PORT]
        for k in self.urdf_parameters:
            if k in d:
                self.urdf_parameters[k] = d[k]

    def get_urdf_parameters(self) -> dict:
        d = {}
        for k, v in self.urdf_parameters.items():
            if v:
                d[k] = v
        return d


class KinovaGen3Dof6(BaseArm):
    MANIPULATOR_MODEL = "kinova_gen3_6dof"


class KinovaGen3Dof7(BaseArm):
    MANIPULATOR_MODEL = "kinova_gen3_7dof"


class KinovaGen3Lite(BaseArm):
    MANIPULATOR_MODEL = "kinova_gen3_lite"


class UniversalRobots(BaseArm):
    MANIPULATOR_MODEL = "universal_robots"

    # Description Variables
    UR_TYPE = 'ur_type'
    INITIAL_POSITIONS = 'initial_positions'
    INITIAL_POSITIONS_FILE = 'initial_positions_file'
    JOINT_LIMITS_PARAMETERS_FILE = 'joint_limits_parameters_file'
    KINEMATICS_PARAMETERS_FILE = 'kinematics_parameters_file'
    PHYSICAL_PARAMETERS_FILE = 'physical_parameters_file'
    VISUAL_PARAMETERS_FILE = 'visual_parameters_file'
    SAFETY_LIMITS = 'safety_limits'
    SAFETY_POS_MARGIN = 'safety_pos_margin'
    SAFETY_K_POSITION = 'safety_k_position'
    # Control Parameters
    GENERATE_ROS2_CONTROL_TAG = 'generate_ros2_control_tag'
    HEADLESS_MODE = 'headless_mode'
    IP_ADDRESS = 'robot_ip'
    SCRIPT_FILENAME = 'script_filename'
    OUTPUT_RECIPE_FILENAME = 'output_recipe_filename'
    INPUT_RECIPE_FILENAME = 'input_recipe_filename'
    REVERSE_IP = 'reverse_ip'
    SCRIPT_COMMAND_PORT = 'script_command_port'
    REVERSE_PORT = 'reverse_port'
    SCRIPT_SENDER_PORT = 'script_sender_port'
    TRAJECTORY_PORT = 'trajectory_port'
    TRANSMISSION_HW_INTERFACE = 'transmission_hw_interface'
    NON_BLOCKING_READ = 'non_blocking_read'
    KEEP_ALIVE_COUNT = 'keep_alive_count'
    # Tool Communication Parameters
    USE_TOOL_COMMUNICATION = 'use_tool_communication'
    TOOL_VOLTAGE = 'tool_voltage'
    TOOL_PARITY = 'tool_parity'
    TOOL_BAUD_RATE = 'tool_baud_rate'
    TOOL_STOP_BITS = 'tool_stop_bits'
    TOOL_RX_IDLE_CHARS = 'tool_rx_idle_chars'
    TOOL_TX_IDLE_CHARS = 'tool_tx_idle_chars'
    TOOL_DEVICE_NAME = 'tool_device_name'
    TOOL_TCP_PORT = 'tool_tcp_port'
    # Simulation Parameters
    USE_FAKE_HARDWARE = 'use_fake_hardware'
    FAKE_SENSOR_COMMANDS = 'fake_sensor_commands'
    SIM_GAZEBO = 'sim_gazebo'
    SIM_IGNITION = 'sim_ignition'

    # URDF Parameters
    URDF_PARAMETERS = {
        UR_TYPE: '',
        INITIAL_POSITIONS: '',
        INITIAL_POSITIONS_FILE: '',
        JOINT_LIMITS_PARAMETERS_FILE: '',
        KINEMATICS_PARAMETERS_FILE: '',
        PHYSICAL_PARAMETERS_FILE: '',
        VISUAL_PARAMETERS_FILE: '',
        SAFETY_LIMITS: '',
        SAFETY_POS_MARGIN: '',
        SAFETY_K_POSITION: '',
        GENERATE_ROS2_CONTROL_TAG: '',
        HEADLESS_MODE: '',
        IP_ADDRESS: '',
        SCRIPT_FILENAME: '',
        OUTPUT_RECIPE_FILENAME: '',
        INPUT_RECIPE_FILENAME: '',
        REVERSE_IP: '',
        SCRIPT_COMMAND_PORT: '',
        REVERSE_PORT: '',
        SCRIPT_SENDER_PORT: '',
        TRAJECTORY_PORT: '',
        TRANSMISSION_HW_INTERFACE: '',
        NON_BLOCKING_READ: '',
        KEEP_ALIVE_COUNT: '',
        USE_TOOL_COMMUNICATION: '',
        TOOL_VOLTAGE: '',
        TOOL_PARITY: '',
        TOOL_BAUD_RATE: '',
        TOOL_STOP_BITS: '',
        TOOL_RX_IDLE_CHARS: '',
        TOOL_TX_IDLE_CHARS: '',
        TOOL_DEVICE_NAME: '',
        TOOL_TCP_PORT: '',
        USE_FAKE_HARDWARE: '',
        FAKE_SENSOR_COMMANDS: '',
        SIM_GAZEBO: '',
        SIM_IGNITION: '',
    }


class Arm():
    KINOVA_GEN3_6DOF = KinovaGen3Dof6.MANIPULATOR_MODEL
    KINOVA_GEN3_7DOF = KinovaGen3Dof7.MANIPULATOR_MODEL
    KINOVA_GEN3_LITE = KinovaGen3Lite.MANIPULATOR_MODEL
    UNIVERSAL_ROBOTS = UniversalRobots.MANIPULATOR_MODEL

    MODEL = {
        KINOVA_GEN3_6DOF: KinovaGen3Dof6,
        KINOVA_GEN3_7DOF: KinovaGen3Dof7,
        KINOVA_GEN3_LITE: KinovaGen3Lite,
        UNIVERSAL_ROBOTS: UniversalRobots,
    }

    @classmethod
    def assert_model(cls, model: str) -> None:
        assert model in cls.MODEL, (
            "Arm model '%s' must be one of: '%s'" % (
                model,
                cls.MODEL.keys()
            )
        )

    def __new__(cls, model: str) -> BaseArm:
        cls.assert_model(model)
        return cls.MODEL[model]()
