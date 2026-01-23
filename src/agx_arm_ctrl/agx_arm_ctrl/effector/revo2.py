#!/usr/bin/env python3
# -*-coding:utf8-*-
from typing import Optional, Dict, Any, List, Literal, TYPE_CHECKING
from dataclasses import dataclass, field

if TYPE_CHECKING:
    from pyAgxArm.effector import EffectorDriver

@dataclass
class HandStatus:
    left_or_right: int = 0      # Hand flag: 01 left hand; 02 right hand
    thumb_tip: int = 0          # Thumb tip motor status: 0 idle; 1 running; 2 blocked
    thumb_base: int = 0         # Thumb base motor status: 0 idle; 1 running; 2 blocked
    index_finger: int = 0       # Index finger motor status: 0 idle; 1 running; 2 blocked
    middle_finger: int = 0      # Middle finger motor status: 0 idle; 1 running; 2 blocked
    ring_finger: int = 0        # Ring finger motor status: 0 idle; 1 running; 2 blocked
    pinky_finger: int = 0       # Pinky finger motor status: 0 idle; 1 running; 2 blocked
    hz: float = 0.0             # Message receiving frequency
    timestamp: float = 0.0      # Timestamp

@dataclass
class FingerPosition:
    thumb_tip: int = 0          # Thumb tip position, range: [0, 100]
    thumb_base: int = 0         # Thumb base position, range: [0, 100]
    index_finger: int = 0       # Index finger position, range: [0, 100]
    middle_finger: int = 0      # Middle finger position, range: [0, 100]
    ring_finger: int = 0        # Ring finger position, range: [0, 100]
    pinky_finger: int = 0       # Pinky finger position, range: [0, 100]
    hz: float = 0.0             # Message receiving frequency
    timestamp: float = 0.0      # Timestamp

@dataclass
class FingerSpeed:
    thumb_tip: int = 0          # Thumb tip speed, range: [-100, 100]
    thumb_base: int = 0         # Thumb base speed, range: [-100, 100]
    index_finger: int = 0       # Index finger speed, range: [-100, 100]
    middle_finger: int = 0      # Middle finger speed, range: [-100, 100]
    ring_finger: int = 0        # Ring finger speed, range: [-100, 100]
    pinky_finger: int = 0       # Pinky finger speed, range: [-100, 100]
    hz: float = 0.0             # Message receiving frequency
    timestamp: float = 0.0      # Timestamp

@dataclass
class FingerCurrent:
    thumb_tip: int = 0          # Thumb tip current, range: [-100, 100]
    thumb_base: int = 0         # Thumb base current, range: [-100, 100]
    index_finger: int = 0       # Index finger current, range: [-100, 100]
    middle_finger: int = 0      # Middle finger current, range: [-100, 100]
    ring_finger: int = 0        # Ring finger current, range: [-100, 100]
    pinky_finger: int = 0       # Pinky finger current, range: [-100, 100]
    hz: float = 0.0             # Message receiving frequency
    timestamp: float = 0.0      # Timestamp


class Revo2Wrapper:

    # Finger name list
    FINGER_NAMES: List[str] = [
        'thumb_tip', 'thumb_base',
        'index_finger', 'middle_finger',
        'ring_finger', 'pinky_finger'
    ]
    
    # Position/current/speed control parameter ranges
    POSITION_MIN: int = 0
    POSITION_MAX: int = 100
    SPEED_MIN: int = -100
    SPEED_MAX: int = 100
    CURRENT_MIN: int = -100
    CURRENT_MAX: int = 100
    TIME_MIN: int = 0
    TIME_MAX: int = 255  # Unit: 10ms
    
    # Motor status constants
    MOTOR_STATUS_IDLE: int = 0      # Idle
    MOTOR_STATUS_RUNNING: int = 1   # Running
    MOTOR_STATUS_BLOCKED: int = 2   # Blocked
    
    # Hand flags
    HAND_LEFT: int = 1
    HAND_RIGHT: int = 2
    
    def __init__(self, agx_arm):
        self._agx_arm = agx_arm
        self._effector: Optional["EffectorDriver"] = None
        self._initialized: bool = False
    
    def initialize(self) -> bool:
        if self._initialized:
            return True
        
        try:
            self._effector = self._agx_arm.init_effector(
                self._agx_arm.EFFECTOR.REVO2
            )
            self._initialized = True
            return True
        except Exception as e:
            print(f"[Revo2Wrapper] Initialization failed: {e}")
            return False
    
    def is_ok(self) -> bool:
        if not self._initialized or self._effector is None:
            return False
        return self._effector.is_ok()
    
    def get_fps(self) -> float:
        if not self._initialized or self._effector is None:
            return 0.0
        return self._effector.get_fps()
    
    def get_status(self) -> Optional[HandStatus]:
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
    
    def _validate_position(self, value: int, name: str) -> None:
        """Validate position parameter range"""
        if not (self.POSITION_MIN <= value <= self.POSITION_MAX):
            raise ValueError(
                f"{name} position must be in range [{self.POSITION_MIN}, {self.POSITION_MAX}], "
                f"current value: {value}"
            )
    
    def _validate_speed(self, value: int, name: str) -> None:
        """Validate speed parameter range"""
        if not (self.SPEED_MIN <= value <= self.SPEED_MAX):
            raise ValueError(
                f"{name} speed must be in range [{self.SPEED_MIN}, {self.SPEED_MAX}], "
                f"current value: {value}"
            )
    
    def _validate_current(self, value: int, name: str) -> None:
        """Validate current parameter range"""
        if not (self.CURRENT_MIN <= value <= self.CURRENT_MAX):
            raise ValueError(
                f"{name} current must be in range [{self.CURRENT_MIN}, {self.CURRENT_MAX}], "
                f"current value: {value}"
            )
    
    def _validate_time(self, value: int, name: str) -> None:
        """Validate time parameter range"""
        if not (self.TIME_MIN <= value <= self.TIME_MAX):
            raise ValueError(
                f"{name} time must be in range [{self.TIME_MIN}, {self.TIME_MAX}], "
                f"current value: {value}"
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
        if not self._initialized or self._effector is None:
            return False
        
        # Parameter validation
        self._validate_position(thumb_tip, 'Thumb tip')
        self._validate_position(thumb_base, 'Thumb base')
        self._validate_position(index_finger, 'Index finger')
        self._validate_position(middle_finger, 'Middle finger')
        self._validate_position(ring_finger, 'Ring finger')
        self._validate_position(pinky_finger, 'Pinky finger')
        
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
            print(f"[Revo2Wrapper] Position control failed: {e}")
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
        if not self._initialized or self._effector is None:
            return False
        
        # Parameter validation
        self._validate_speed(thumb_tip, 'Thumb tip')
        self._validate_speed(thumb_base, 'Thumb base')
        self._validate_speed(index_finger, 'Index finger')
        self._validate_speed(middle_finger, 'Middle finger')
        self._validate_speed(ring_finger, 'Ring finger')
        self._validate_speed(pinky_finger, 'Pinky finger')
        
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
            print(f"[Revo2Wrapper] Speed control failed: {e}")
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
        if not self._initialized or self._effector is None:
            return False
        
        # Parameter validation
        self._validate_current(thumb_tip, 'Thumb tip')
        self._validate_current(thumb_base, 'Thumb base')
        self._validate_current(index_finger, 'Index finger')
        self._validate_current(middle_finger, 'Middle finger')
        self._validate_current(ring_finger, 'Ring finger')
        self._validate_current(pinky_finger, 'Pinky finger')
        
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
            print(f"[Revo2Wrapper] Current control failed: {e}")
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
        if not self._initialized or self._effector is None:
            return False
        
        if mode not in ['pos', 'time']:
            raise ValueError(f"mode must be 'pos' or 'time', current value: {mode}")
        
        # Select validation method based on mode
        if mode == 'pos':
            self._validate_position(thumb_tip, 'Thumb tip')
            self._validate_position(thumb_base, 'Thumb base')
            self._validate_position(index_finger, 'Index finger')
            self._validate_position(middle_finger, 'Middle finger')
            self._validate_position(ring_finger, 'Ring finger')
            self._validate_position(pinky_finger, 'Pinky finger')
        else:  # time mode
            self._validate_time(thumb_tip, 'Thumb tip')
            self._validate_time(thumb_base, 'Thumb base')
            self._validate_time(index_finger, 'Index finger')
            self._validate_time(middle_finger, 'Middle finger')
            self._validate_time(ring_finger, 'Ring finger')
            self._validate_time(pinky_finger, 'Pinky finger')
        
        # TODO:
        print(f"[Revo2Wrapper] Position/time hybrid control: {mode}")
        print(f"thumb_tip: {thumb_tip}, thumb_base: {thumb_base}, index_finger: {index_finger}, middle_finger: {middle_finger}, ring_finger: {ring_finger}, pinky_finger: {pinky_finger}")

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
            print(f"[Revo2Wrapper] Position/time hybrid control failed: {e}")
            return False
    
    def is_hand_left(self) -> bool:
        status = self.get_status()
        if status is None:
            return False
        return status.left_or_right == self.HAND_LEFT
    
    def is_hand_right(self) -> bool:
        status = self.get_status()
        if status is None:
            return False
        return status.left_or_right == self.HAND_RIGHT
