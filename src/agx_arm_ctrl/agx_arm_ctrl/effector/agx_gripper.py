#!/usr/bin/env python3
# -*-coding:utf8-*-
"""
AgxGripper 末端执行器封装模块

本模块封装了AgxGripper夹爪的控制和状态读取功能，
供agx_arm_ctrl_single_node.py调用。

功能包括：
- 夹爪初始化
- 状态读取（宽度、夹持力、驱动状态等）
- 夹爪控制（移动、禁用、标定）
- 示教器参数配置
"""

from typing import Optional, Dict, Any, TYPE_CHECKING
from dataclasses import dataclass

if TYPE_CHECKING:
    from pyAgxArm.effector import EffectorDriver


@dataclass
class GripperStatus:
    """夹爪状态数据类"""
    width: float = 0.0          # 当前夹爪开口宽度，单位：m
    force: float = 0.0          # 当前夹持力，单位：N
    voltage_too_low: bool = False       # 电压过低
    motor_overheating: bool = False     # 电机过温
    driver_overcurrent: bool = False    # 驱动过流
    driver_overheating: bool = False    # 驱动过温
    sensor_status: bool = False         # 传感器状态
    driver_error_status: bool = False   # 驱动错误状态
    driver_enable_status: bool = False  # 驱动使能状态
    homing_status: bool = False         # 回零/零位状态
    hz: float = 0.0             # 消息接收频率
    timestamp: float = 0.0      # 时间戳


@dataclass
class GripperCtrlStatus:
    """夹爪控制状态数据类"""
    width: float = 0.0          # 当前夹爪开口宽度，单位：m
    force: float = 0.0          # 当前夹持力，单位：N
    status_code: int = 0        # 状态码
    set_zero: int = 0           # 回零/置零字段
    hz: float = 0.0             # 消息接收频率
    timestamp: float = 0.0      # 时间戳


@dataclass
class GripperTeachingParam:
    """夹爪示教器参数数据类"""
    teaching_range_per: int = 100   # 示教范围（百分比），范围：[100, 200]
    max_range_config: float = 0.0   # 最大行程配置，单位：m，仅支持：0/0.07/0.1
    teaching_friction: int = 1      # 示教摩擦，范围：1..10
    hz: float = 0.0
    timestamp: float = 0.0


class AgxGripperWrapper:
    """
    AgxGripper夹爪封装类
    
    该类封装了pyAgxArm库中AgxGripper的所有功能，
    提供更简洁的接口供ROS节点调用。
    
    使用示例：
        gripper = AgxGripperWrapper(agx_arm)
        gripper.initialize()
        gripper.move(width=0.05, force=1.0)
    """
    
    # 夹爪控制参数范围
    WIDTH_MIN: float = 0.0      # 最小宽度，单位：m
    WIDTH_MAX: float = 0.1      # 最大宽度，单位：m
    FORCE_MIN: float = 0.0      # 最小力，单位：N
    FORCE_MAX: float = 3.0      # 最大力，单位：N
    
    def __init__(self, agx_arm):
        self._agx_arm = agx_arm
        self._effector: Optional["EffectorDriver"] = None
        self._initialized: bool = False
    
    def initialize(self) -> bool:
        if self._initialized:
            return True
        
        try:
            self._effector = self._agx_arm.init_effector(
                self._agx_arm.EFFECTOR.AGX_GRIPPER
            )
            self._initialized = True
            return True
        except Exception as e:
            print(f"[AgxGripperWrapper] 初始化失败: {e}")
            return False
    
    @property
    def is_initialized(self) -> bool:
        return self._initialized
    
    @property
    def effector(self) -> Optional["EffectorDriver"]:
        return self._effector
    
    def is_ok(self) -> bool:
        if not self._initialized or self._effector is None:
            return False
        return self._effector.is_ok()
    
    def get_fps(self) -> float:
        if not self._initialized or self._effector is None:
            return 0.0
        return self._effector.get_fps()
    
    def get_status(self) -> Optional[GripperStatus]:
        if not self._initialized or self._effector is None:
            return None
        
        gs = self._effector.get_gripper_status()
        if gs is None:
            return None
        
        status = GripperStatus(
            width=gs.msg.width,
            force=gs.msg.force,
            voltage_too_low=gs.msg.foc_status.voltage_too_low,
            motor_overheating=gs.msg.foc_status.motor_overheating,
            driver_overcurrent=gs.msg.foc_status.driver_overcurrent,
            driver_overheating=gs.msg.foc_status.driver_overheating,
            sensor_status=gs.msg.foc_status.sensor_status,
            driver_error_status=gs.msg.foc_status.driver_error_status,
            driver_enable_status=gs.msg.foc_status.driver_enable_status,
            homing_status=gs.msg.foc_status.homing_status,
            hz=gs.hz,
            timestamp=gs.timestamp
        )
        return status
    
    def get_ctrl_status(self) -> Optional[GripperCtrlStatus]:
        if not self._initialized or self._effector is None:
            return None
        
        gcs = self._effector.get_gripper_ctrl_states()
        if gcs is None:
            return None
        
        ctrl_status = GripperCtrlStatus(
            width=gcs.msg.width,
            force=gcs.msg.force,
            status_code=gcs.msg.status_code,
            set_zero=gcs.msg.set_zero,
            hz=gcs.hz,
            timestamp=gcs.timestamp
        )
        return ctrl_status
    
    def get_teaching_param(
        self, 
        timeout: float = 1.0, 
        min_interval: float = 1.0
    ) -> Optional[GripperTeachingParam]:
        if not self._initialized or self._effector is None:
            return None
        
        param = self._effector.get_gripper_teaching_pendant_param(
            timeout=timeout, 
            min_interval=min_interval
        )
        if param is None:
            return None
        
        teaching_param = GripperTeachingParam(
            teaching_range_per=param.msg.teaching_range_per,
            max_range_config=param.msg.max_range_config,
            teaching_friction=param.msg.teaching_friction,
            hz=param.hz,
            timestamp=param.timestamp
        )
        return teaching_param
    
    def move(self, width: float = 0.0, force: float = 1.0) -> bool:
        if not self._initialized or self._effector is None:
            return False
        
        # 参数范围校验
        if not (self.WIDTH_MIN <= width <= self.WIDTH_MAX):
            raise ValueError(
                f"width必须在[{self.WIDTH_MIN}, {self.WIDTH_MAX}]范围内，当前值：{width}"
            )
        if not (self.FORCE_MIN <= force <= self.FORCE_MAX):
            raise ValueError(
                f"force必须在[{self.FORCE_MIN}, {self.FORCE_MAX}]范围内，当前值：{force}"
            )
        
        try:
            self._effector.move_gripper(width=width, force=force)
            return True
        except Exception as e:
            print(f"[AgxGripperWrapper] 控制夹爪失败: {e}")
            return False
    
    def open(self, width: float = 0.1, force: float = 1.0) -> bool:
        return self.move(width=width, force=force)
    
    def close(self, force: float = 1.0) -> bool:
        return self.move(width=0.0, force=force)
    
    def disable(self) -> bool:
        if not self._initialized or self._effector is None:
            return False
        
        return self._effector.disable_gripper()
    
    def calibrate(self, timeout: float = 1.0) -> bool:
        if not self._initialized or self._effector is None:
            return False
        
        return self._effector.calibrate_gripper(timeout=timeout)
    
    def set_teaching_param(
        self,
        teaching_range_per: int = 100,
        max_range_config: float = 0.0,
        teaching_friction: int = 1,
        timeout: float = 1.0
    ) -> bool:
        if not self._initialized or self._effector is None:
            return False
        
        return self._effector.set_gripper_teaching_pendant_param(
            teaching_range_per=teaching_range_per,
            max_range_config=max_range_config,
            teaching_friction=teaching_friction,
            timeout=timeout
        )
    
    def get_status_dict(self) -> Dict[str, Any]:
        """
        获取夹爪状态字典（便于ROS消息发布）
        
        Returns:
            Dict: 包含夹爪状态的字典
        """
        status = self.get_status()
        if status is None:
            return {}
        
        return {
            'width': status.width,
            'force': status.force,
            'driver_enable_status': status.driver_enable_status,
            'homing_status': status.homing_status,
            'voltage_too_low': status.voltage_too_low,
            'motor_overheating': status.motor_overheating,
            'driver_overcurrent': status.driver_overcurrent,
            'driver_overheating': status.driver_overheating,
            'sensor_status': status.sensor_status,
            'driver_error_status': status.driver_error_status,
            'hz': status.hz,
            'timestamp': status.timestamp
        }

