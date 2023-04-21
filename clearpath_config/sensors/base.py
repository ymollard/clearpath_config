from clearpath_config.common import Accessory, IndexedAccessory
from typing import List


class BaseSensor(IndexedAccessory):
    """
    Base Sensor Class
     - inherits from Accessory.
     - contains all common parameters shared by all sensors.
    """
    SENSOR_TYPE = "generic"
    SENSOR_MODEL = "base"
    TOPIC = "base"
    URDF_ENABLED = True
    LAUNCH_ENABLED = True
    ROS_PARAMETERS = {}

    def __init__(
            self,
            idx: int = None,
            name: str = None,
            topic: str = TOPIC,
            urdf_enabled: bool = URDF_ENABLED,
            launch_enabled: bool = LAUNCH_ENABLED,
            ros_parameters: str = ROS_PARAMETERS,
            parent: str = Accessory.PARENT,
            xyz: List[float] = Accessory.XYZ,
            rpy: List[float] = Accessory.RPY,
            ) -> None:
        # Topic:
        # - should be automatically determined by the sensor's index
        # - should match the Clearpath API
        self.topic = str()
        self.set_topic(topic)
        # URDF Enable
        # - enables the sensor description in the generated URDF
        self.urdf_enabled = True
        self.enable_urdf() if urdf_enabled else self.disable_urdf()
        # Launch Enable
        # - enables the sensor launch in the generated launch
        self.launch_enabled = True
        self.enable_launch if launch_enabled else self.disable_launch()
        # ROS Parameters
        # - dictionary with parameters for launch file
        self.ros_parameters = {}
        self.set_ros_parameters(ros_parameters)
        super().__init__(idx, name, parent, xyz, rpy)

    @classmethod
    def get_sensor_type(cls) -> str:
        return cls.SENSOR_TYPE

    @classmethod
    def get_sensor_model(cls) -> str:
        return cls.SENSOR_MODEL

    @classmethod
    def get_name_from_idx(cls, idx: int) -> str:
        return "%s_%s" % (
            cls.get_sensor_type(),
            idx
        )

    @classmethod
    def get_topic_from_idx(cls, idx: int) -> str:
        return "%s/%s" % (
            cls.get_name_from_idx(idx),
            cls.TOPIC
        )

    def set_idx(self, idx: int) -> None:
        super().set_idx(idx)
        self.topic = self.get_topic_from_idx(idx)

    def get_topic(self) -> str:
        return self.topic

    def set_topic(self, topic: str) -> None:
        assert isinstance(topic, str), (
            "Topic '%s' of type '%s', expected 'str'" % (topic, type(topic))
        )
        assert " " not in topic, (
            "Topic '%s' contains empty spaces." % topic
        )
        self.topic = topic

    def enable_urdf(self) -> None:
        self.urdf_enabled = True

    def disable_urdf(self) -> None:
        self.urdf_enabled = False

    def set_urdf_enabled(self, enabled: bool) -> None:
        self.enable_urdf() if enabled else self.disable_urdf()

    def get_urdf_enabled(self) -> bool:
        return self.urdf_enabled

    def enable_launch(self) -> None:
        self.launch_enabled = True

    def disable_launch(self) -> None:
        self.launch_enabled = False

    def set_launch_enabled(self, enabled: bool) -> None:
        self.enable_launch() if enabled else self.disable_launch()

    def get_launch_enabled(self) -> bool:
        return self.launch_enabled

    def update_ros_parameters(self) -> None:
        pass

    def set_ros_parameters(self, ros_parameters: dict) -> None:
        assert isinstance(ros_parameters, dict), (
            "ROS paramaters must be a dictionary")
        self.ros_parameters = ros_parameters

    def get_ros_parameters(self) -> dict:
        self.update_ros_parameters()
        return self.ros_parameters
