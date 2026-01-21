#!/usr/bin/env python3
# -*-coding:utf8-*-
"""
Revo2 灵巧手末端执行器封装模块

本模块封装了Revo2灵巧手的控制和状态读取功能，
供agx_arm_ctrl_single_node.py调用。

功能包括：
- 灵巧手初始化
- 状态读取（各指位置、速度、电流、运行状态）
- 灵巧手控制（位置控制、速度控制、电流控制、位置/时间混合控制）
"""

from typing import Optional, Dict, Any, List, Literal, TYPE_CHECKING
from dataclasses import dataclass, field

if TYPE_CHECKING:
    from pyAgxArm.effector import EffectorDriver


@dataclass
class HandStatus:
    """灵巧手状态数据类"""
    left_or_right: int = 0      # 左右手标志：01 左手；02 右手
    thumb_tip: int = 0          # 拇指尖电机状态：0 空闲；1 运行；2 堵转
    thumb_base: int = 0         # 拇指根电机状态：0 空闲；1 运行；2 堵转
    index_finger: int = 0       # 食指电机状态：0 空闲；1 运行；2 堵转
    middle_finger: int = 0      # 中指电机状态：0 空闲；1 运行；2 堵转
    ring_finger: int = 0        # 无名指电机状态：0 空闲；1 运行；2 堵转
    pinky_finger: int = 0       # 小指电机状态：0 空闲；1 运行；2 堵转
    hz: float = 0.0             # 消息接收频率
    timestamp: float = 0.0      # 时间戳


@dataclass
class FingerPosition:
    """各指位置数据类"""
    thumb_tip: int = 0          # 拇指尖位置，范围：[0, 100]
    thumb_base: int = 0         # 拇指根位置，范围：[0, 100]
    index_finger: int = 0       # 食指位置，范围：[0, 100]
    middle_finger: int = 0      # 中指位置，范围：[0, 100]
    ring_finger: int = 0        # 无名指位置，范围：[0, 100]
    pinky_finger: int = 0       # 小指位置，范围：[0, 100]
    hz: float = 0.0             # 消息接收频率
    timestamp: float = 0.0      # 时间戳


@dataclass
class FingerSpeed:
    """各指速度数据类"""
    thumb_tip: int = 0          # 拇指尖速度，范围：[-100, 100]
    thumb_base: int = 0         # 拇指根速度，范围：[-100, 100]
    index_finger: int = 0       # 食指速度，范围：[-100, 100]
    middle_finger: int = 0      # 中指速度，范围：[-100, 100]
    ring_finger: int = 0        # 无名指速度，范围：[-100, 100]
    pinky_finger: int = 0       # 小指速度，范围：[-100, 100]
    hz: float = 0.0             # 消息接收频率
    timestamp: float = 0.0      # 时间戳


@dataclass
class FingerCurrent:
    """各指电流数据类"""
    thumb_tip: int = 0          # 拇指尖电流，范围：[-100, 100]
    thumb_base: int = 0         # 拇指根电流，范围：[-100, 100]
    index_finger: int = 0       # 食指电流，范围：[-100, 100]
    middle_finger: int = 0      # 中指电流，范围：[-100, 100]
    ring_finger: int = 0        # 无名指电流，范围：[-100, 100]
    pinky_finger: int = 0       # 小指电流，范围：[-100, 100]
    hz: float = 0.0             # 消息接收频率
    timestamp: float = 0.0      # 时间戳


class Revo2Wrapper:
    """
    Revo2灵巧手封装类
    
    该类封装了pyAgxArm库中Revo2灵巧手的所有功能，
    提供更简洁的接口供ROS节点调用。
    
    手指命名说明：
    - thumb_tip: 拇指尖
    - thumb_base: 拇指根
    - index_finger: 食指
    - middle_finger: 中指
    - ring_finger: 无名指
    - pinky_finger: 小指
    
    使用示例：
        hand = Revo2Wrapper(agx_arm)
        hand.initialize()
        hand.position_ctrl(thumb_tip=50, index_finger=100)
    """
    
    # 手指名称列表（用于遍历）
    FINGER_NAMES: List[str] = [
        'thumb_tip', 'thumb_base', 
        'index_finger', 'middle_finger', 
        'ring_finger', 'pinky_finger'
    ]
    
    # 位置/电流/速度控制参数范围
    POSITION_MIN: int = 0
    POSITION_MAX: int = 100
    SPEED_MIN: int = -100
    SPEED_MAX: int = 100
    CURRENT_MIN: int = -100
    CURRENT_MAX: int = 100
    TIME_MIN: int = 0
    TIME_MAX: int = 255  # 单位10ms
    
    # 电机状态常量
    MOTOR_STATUS_IDLE: int = 0      # 空闲
    MOTOR_STATUS_RUNNING: int = 1   # 运行
    MOTOR_STATUS_BLOCKED: int = 2   # 堵转
    
    # 左右手标志
    HAND_LEFT: int = 1
    HAND_RIGHT: int = 2
    
    def __init__(self, agx_arm):
        """
        初始化Revo2封装类
        
        Args:
            agx_arm: AgxArm机械臂实例（由AgxArmFactory.create_arm创建）
        """
        self._agx_arm = agx_arm
        self._effector: Optional["EffectorDriver"] = None
        self._initialized: bool = False
    
    def initialize(self) -> bool:
        """
        初始化末端执行器
        
        注意：
        1. init_effector函数只能执行一次，无法初始化两个执行器
        2. 最好在robot.connect()前进行执行器的创建
        
        Returns:
            bool: 初始化是否成功
        """
        if self._initialized:
            return True
        
        try:
            self._effector = self._agx_arm.init_effector(
                self._agx_arm.EFFECTOR.REVO2
            )
            self._initialized = True
            return True
        except Exception as e:
            print(f"[Revo2Wrapper] 初始化失败: {e}")
            return False
    
    @property
    def is_initialized(self) -> bool:
        """返回是否已初始化"""
        return self._initialized
    
    @property
    def effector(self) -> Optional["EffectorDriver"]:
        """返回底层执行器实例"""
        return self._effector
    
    # ==================== 通用状态相关 ====================
    
    def is_ok(self) -> bool:
        """
        检查末端执行器通信是否正常
        
        用于判断末端执行器数据接收是否正常。
        该值由SDK内部的数据监控逻辑根据"最近一段时间是否持续收不到数据"计算得出。
        
        Returns:
            bool: 通信正常返回True，否则返回False
        """
        if not self._initialized or self._effector is None:
            return False
        return self._effector.is_ok()
    
    def get_fps(self) -> float:
        """
        获取执行器数据接收频率
        
        Returns:
            float: 接收频率，单位：Hz
        """
        if not self._initialized or self._effector is None:
            return 0.0
        return self._effector.get_fps()
    
    # ==================== 状态读取相关 ====================
    
    def get_hand_status(self) -> Optional[HandStatus]:
        """
        读取灵巧手各指运行状态
        
        包含左右手标志、各指电机状态（空闲/运行/堵转）。
        
        Returns:
            HandStatus: 手状态，如果读取失败返回None
        """
        if not self._initialized or self._effector is None:
            return None
        
        hs = self._effector.get_hand_status()
        if hs is None:
            return None
        
        status = HandStatus(
            left_or_right=hs.msg.left_or_right,
            thumb_tip=hs.msg.thumb_tip,
            thumb_base=hs.msg.thumb_base,
            index_finger=hs.msg.index_finger,
            middle_finger=hs.msg.middle_finger,
            ring_finger=hs.msg.ring_finger,
            pinky_finger=hs.msg.pinky_finger,
            hz=hs.hz,
            timestamp=hs.timestamp
        )
        return status
    
    def get_finger_position(self) -> Optional[FingerPosition]:
        """
        读取各指位置反馈
        
        各指位置范围：[0, 100]
        
        Returns:
            FingerPosition: 各指位置，如果读取失败返回None
        """
        if not self._initialized or self._effector is None:
            return None
        
        fp = self._effector.get_finger_pos()
        if fp is None:
            return None
        
        position = FingerPosition(
            thumb_tip=fp.msg.thumb_tip,
            thumb_base=fp.msg.thumb_base,
            index_finger=fp.msg.index_finger,
            middle_finger=fp.msg.middle_finger,
            ring_finger=fp.msg.ring_finger,
            pinky_finger=fp.msg.pinky_finger,
            hz=fp.hz,
            timestamp=fp.timestamp
        )
        return position
    
    def get_finger_speed(self) -> Optional[FingerSpeed]:
        """
        读取各指速度反馈
        
        各指速度范围：[-100, 100]
        
        Returns:
            FingerSpeed: 各指速度，如果读取失败返回None
        """
        if not self._initialized or self._effector is None:
            return None
        
        fs = self._effector.get_finger_spd()
        if fs is None:
            return None
        
        speed = FingerSpeed(
            thumb_tip=fs.msg.thumb_tip,
            thumb_base=fs.msg.thumb_base,
            index_finger=fs.msg.index_finger,
            middle_finger=fs.msg.middle_finger,
            ring_finger=fs.msg.ring_finger,
            pinky_finger=fs.msg.pinky_finger,
            hz=fs.hz,
            timestamp=fs.timestamp
        )
        return speed
    
    def get_finger_current(self) -> Optional[FingerCurrent]:
        """
        读取各指电流反馈
        
        各指电流范围：[-100, 100]
        
        Returns:
            FingerCurrent: 各指电流，如果读取失败返回None
        """
        if not self._initialized or self._effector is None:
            return None
        
        fc = self._effector.get_finger_current()
        if fc is None:
            return None
        
        current = FingerCurrent(
            thumb_tip=fc.msg.thumb_tip,
            thumb_base=fc.msg.thumb_base,
            index_finger=fc.msg.index_finger,
            middle_finger=fc.msg.middle_finger,
            ring_finger=fc.msg.ring_finger,
            pinky_finger=fc.msg.pinky_finger,
            hz=fc.hz,
            timestamp=fc.timestamp
        )
        return current
    
    # ==================== 控制相关 ====================
    
    def _validate_position(self, value: int, name: str) -> None:
        """验证位置参数范围"""
        if not (self.POSITION_MIN <= value <= self.POSITION_MAX):
            raise ValueError(
                f"{name}位置必须在[{self.POSITION_MIN}, {self.POSITION_MAX}]范围内，"
                f"当前值：{value}"
            )
    
    def _validate_speed(self, value: int, name: str) -> None:
        """验证速度参数范围"""
        if not (self.SPEED_MIN <= value <= self.SPEED_MAX):
            raise ValueError(
                f"{name}速度必须在[{self.SPEED_MIN}, {self.SPEED_MAX}]范围内，"
                f"当前值：{value}"
            )
    
    def _validate_current(self, value: int, name: str) -> None:
        """验证电流参数范围"""
        if not (self.CURRENT_MIN <= value <= self.CURRENT_MAX):
            raise ValueError(
                f"{name}电流必须在[{self.CURRENT_MIN}, {self.CURRENT_MAX}]范围内，"
                f"当前值：{value}"
            )
    
    def position_ctrl(
        self,
        thumb_tip: int = 0,
        thumb_base: int = 0,
        index_finger: int = 0,
        middle_finger: int = 0,
        ring_finger: int = 0,
        pinky_finger: int = 0
    ) -> bool:
        """
        位置模式控制各手指目标位置
        
        Args:
            thumb_tip: 拇指尖位置，范围：[0, 100]
            thumb_base: 拇指根位置，范围：[0, 100]
            index_finger: 食指位置，范围：[0, 100]
            middle_finger: 中指位置，范围：[0, 100]
            ring_finger: 无名指位置，范围：[0, 100]
            pinky_finger: 小指位置，范围：[0, 100]
        
        Returns:
            bool: 指令发送是否成功
        """
        if not self._initialized or self._effector is None:
            return False
        
        # 参数验证
        self._validate_position(thumb_tip, '拇指尖')
        self._validate_position(thumb_base, '拇指根')
        self._validate_position(index_finger, '食指')
        self._validate_position(middle_finger, '中指')
        self._validate_position(ring_finger, '无名指')
        self._validate_position(pinky_finger, '小指')
        
        try:
            self._effector.position_ctrl(
                thumb_tip=thumb_tip,
                thumb_base=thumb_base,
                index_finger=index_finger,
                middle_finger=middle_finger,
                ring_finger=ring_finger,
                pinky_finger=pinky_finger
            )
            return True
        except Exception as e:
            print(f"[Revo2Wrapper] 位置控制失败: {e}")
            return False
    
    def speed_ctrl(
        self,
        thumb_tip: int = 0,
        thumb_base: int = 0,
        index_finger: int = 0,
        middle_finger: int = 0,
        ring_finger: int = 0,
        pinky_finger: int = 0
    ) -> bool:
        """
        速度模式控制各手指目标速度
        
        Args:
            thumb_tip: 拇指尖速度，范围：[-100, 100]
            thumb_base: 拇指根速度，范围：[-100, 100]
            index_finger: 食指速度，范围：[-100, 100]
            middle_finger: 中指速度，范围：[-100, 100]
            ring_finger: 无名指速度，范围：[-100, 100]
            pinky_finger: 小指速度，范围：[-100, 100]
        
        Returns:
            bool: 指令发送是否成功
        """
        if not self._initialized or self._effector is None:
            return False
        
        # 参数验证
        self._validate_speed(thumb_tip, '拇指尖')
        self._validate_speed(thumb_base, '拇指根')
        self._validate_speed(index_finger, '食指')
        self._validate_speed(middle_finger, '中指')
        self._validate_speed(ring_finger, '无名指')
        self._validate_speed(pinky_finger, '小指')
        
        try:
            self._effector.speed_ctrl(
                thumb_tip=thumb_tip,
                thumb_base=thumb_base,
                index_finger=index_finger,
                middle_finger=middle_finger,
                ring_finger=ring_finger,
                pinky_finger=pinky_finger
            )
            return True
        except Exception as e:
            print(f"[Revo2Wrapper] 速度控制失败: {e}")
            return False
    
    def current_ctrl(
        self,
        thumb_tip: int = 0,
        thumb_base: int = 0,
        index_finger: int = 0,
        middle_finger: int = 0,
        ring_finger: int = 0,
        pinky_finger: int = 0
    ) -> bool:
        """
        电流模式控制各手指目标电流
        
        Args:
            thumb_tip: 拇指尖电流，范围：[-100, 100]
            thumb_base: 拇指根电流，范围：[-100, 100]
            index_finger: 食指电流，范围：[-100, 100]
            middle_finger: 中指电流，范围：[-100, 100]
            ring_finger: 无名指电流，范围：[-100, 100]
            pinky_finger: 小指电流，范围：[-100, 100]
        
        Returns:
            bool: 指令发送是否成功
        """
        if not self._initialized or self._effector is None:
            return False
        
        # 参数验证
        self._validate_current(thumb_tip, '拇指尖')
        self._validate_current(thumb_base, '拇指根')
        self._validate_current(index_finger, '食指')
        self._validate_current(middle_finger, '中指')
        self._validate_current(ring_finger, '无名指')
        self._validate_current(pinky_finger, '小指')
        
        try:
            self._effector.current_ctrl(
                thumb_tip=thumb_tip,
                thumb_base=thumb_base,
                index_finger=index_finger,
                middle_finger=middle_finger,
                ring_finger=ring_finger,
                pinky_finger=pinky_finger
            )
            return True
        except Exception as e:
            print(f"[Revo2Wrapper] 电流控制失败: {e}")
            return False
    
    def position_time_ctrl(
        self,
        mode: Literal['pos', 'time'] = 'pos',
        thumb_tip: int = 0,
        thumb_base: int = 0,
        index_finger: int = 0,
        middle_finger: int = 0,
        ring_finger: int = 0,
        pinky_finger: int = 0
    ) -> bool:
        """
        位置/时间混合控制
        
        使用方法：
        1. 先用 mode="pos" 下发目标位置
        2. 再用 mode="time" 下发到位时间（单位10ms）
        注意：两条消息的间隔不应超过50ms
        
        Args:
            mode: 控制模式，'pos'为位置模式(0~100)，'time'为时间模式(单位10ms，范围0~255)
            thumb_tip: 拇指尖位置/时间
            thumb_base: 拇指根位置/时间
            index_finger: 食指位置/时间
            middle_finger: 中指位置/时间
            ring_finger: 无名指位置/时间
            pinky_finger: 小指位置/时间
        
        Returns:
            bool: 指令发送是否成功
        
        示例：
            # 拇指尖移动到位置100，然后设置2秒到位（200 * 10ms）
            hand.position_time_ctrl(mode="pos", thumb_tip=100)
            time.sleep(0.02)  # 建议短间隔，确保 < 50ms
            hand.position_time_ctrl(mode="time", thumb_tip=200)
        """
        if not self._initialized or self._effector is None:
            return False
        
        if mode not in ['pos', 'time']:
            raise ValueError(f"mode必须是'pos'或'time'，当前值：{mode}")
        
        # 根据模式选择验证方法
        if mode == 'pos':
            self._validate_position(thumb_tip, '拇指尖')
            self._validate_position(thumb_base, '拇指根')
            self._validate_position(index_finger, '食指')
            self._validate_position(middle_finger, '中指')
            self._validate_position(ring_finger, '无名指')
            self._validate_position(pinky_finger, '小指')
        else:  # time mode
            for name, value in [
                ('拇指尖', thumb_tip), ('拇指根', thumb_base),
                ('食指', index_finger), ('中指', middle_finger),
                ('无名指', ring_finger), ('小指', pinky_finger)
            ]:
                if not (self.TIME_MIN <= value <= self.TIME_MAX):
                    raise ValueError(
                        f"{name}时间必须在[{self.TIME_MIN}, {self.TIME_MAX}]范围内，"
                        f"当前值：{value}"
                    )
        
        try:
            self._effector.position_time_ctrl(
                mode=mode,
                thumb_tip=thumb_tip,
                thumb_base=thumb_base,
                index_finger=index_finger,
                middle_finger=middle_finger,
                ring_finger=ring_finger,
                pinky_finger=pinky_finger
            )
            return True
        except Exception as e:
            print(f"[Revo2Wrapper] 位置/时间混合控制失败: {e}")
            return False
    
    # ==================== 便捷控制方法 ====================
    
    def open_all(self) -> bool:
        """
        张开所有手指（所有手指位置设为0）
        
        Returns:
            bool: 指令发送是否成功
        """
        return self.position_ctrl(
            thumb_tip=0,
            thumb_base=0,
            index_finger=0,
            middle_finger=0,
            ring_finger=0,
            pinky_finger=0
        )
    
    def close_all(self) -> bool:
        """
        闭合所有手指（所有手指位置设为100）
        
        Returns:
            bool: 指令发送是否成功
        """
        return self.position_ctrl(
            thumb_tip=100,
            thumb_base=100,
            index_finger=100,
            middle_finger=100,
            ring_finger=100,
            pinky_finger=100
        )
    
    def set_all_fingers(self, position: int) -> bool:
        """
        将所有手指设置到同一位置
        
        Args:
            position: 目标位置，范围：[0, 100]
        
        Returns:
            bool: 指令发送是否成功
        """
        self._validate_position(position, '所有手指')
        return self.position_ctrl(
            thumb_tip=position,
            thumb_base=position,
            index_finger=position,
            middle_finger=position,
            ring_finger=position,
            pinky_finger=position
        )
    
    def grasp(self, thumb_pos: int = 80, fingers_pos: int = 100) -> bool:
        """
        执行抓取动作
        
        Args:
            thumb_pos: 拇指位置，默认80
            fingers_pos: 其他手指位置，默认100
        
        Returns:
            bool: 指令发送是否成功
        """
        return self.position_ctrl(
            thumb_tip=thumb_pos,
            thumb_base=thumb_pos,
            index_finger=fingers_pos,
            middle_finger=fingers_pos,
            ring_finger=fingers_pos,
            pinky_finger=fingers_pos
        )
    
    def point(self) -> bool:
        """
        执行指向手势（食指伸出，其他手指闭合）
        
        Returns:
            bool: 指令发送是否成功
        """
        return self.position_ctrl(
            thumb_tip=100,
            thumb_base=100,
            index_finger=0,
            middle_finger=100,
            ring_finger=100,
            pinky_finger=100
        )
    
    def victory(self) -> bool:
        """
        执行胜利手势（食指和中指伸出）
        
        Returns:
            bool: 指令发送是否成功
        """
        return self.position_ctrl(
            thumb_tip=100,
            thumb_base=100,
            index_finger=0,
            middle_finger=0,
            ring_finger=100,
            pinky_finger=100
        )
    
    def thumb_up(self) -> bool:
        """
        执行竖大拇指手势
        
        Returns:
            bool: 指令发送是否成功
        """
        return self.position_ctrl(
            thumb_tip=0,
            thumb_base=0,
            index_finger=100,
            middle_finger=100,
            ring_finger=100,
            pinky_finger=100
        )
    
    # ==================== ROS发布辅助方法 ====================
    
    def get_finger_position_list(self) -> List[int]:
        """
        获取各指位置列表（便于ROS消息发布）
        
        返回顺序：[thumb_tip, thumb_base, index_finger, middle_finger, ring_finger, pinky_finger]
        
        Returns:
            List[int]: 各指位置列表，如果读取失败返回空列表
        """
        pos = self.get_finger_position()
        if pos is None:
            return []
        
        return [
            pos.thumb_tip, pos.thumb_base,
            pos.index_finger, pos.middle_finger,
            pos.ring_finger, pos.pinky_finger
        ]
    
    def get_status_dict(self) -> Dict[str, Any]:
        """
        获取灵巧手状态字典（便于ROS消息发布）
        
        Returns:
            Dict: 包含灵巧手状态的字典
        """
        status = self.get_hand_status()
        position = self.get_finger_position()
        
        result = {}
        
        if status is not None:
            result.update({
                'left_or_right': status.left_or_right,
                'motor_status': {
                    'thumb_tip': status.thumb_tip,
                    'thumb_base': status.thumb_base,
                    'index_finger': status.index_finger,
                    'middle_finger': status.middle_finger,
                    'ring_finger': status.ring_finger,
                    'pinky_finger': status.pinky_finger,
                },
                'status_hz': status.hz,
                'status_timestamp': status.timestamp
            })
        
        if position is not None:
            result.update({
                'finger_position': {
                    'thumb_tip': position.thumb_tip,
                    'thumb_base': position.thumb_base,
                    'index_finger': position.index_finger,
                    'middle_finger': position.middle_finger,
                    'ring_finger': position.ring_finger,
                    'pinky_finger': position.pinky_finger,
                },
                'position_hz': position.hz,
                'position_timestamp': position.timestamp
            })
        
        return result
    
    def is_hand_left(self) -> bool:
        """
        判断是否为左手
        
        Returns:
            bool: 是左手返回True
        """
        status = self.get_hand_status()
        if status is None:
            return False
        return status.left_or_right == self.HAND_LEFT
    
    def is_hand_right(self) -> bool:
        """
        判断是否为右手
        
        Returns:
            bool: 是右手返回True
        """
        status = self.get_hand_status()
        if status is None:
            return False
        return status.left_or_right == self.HAND_RIGHT

