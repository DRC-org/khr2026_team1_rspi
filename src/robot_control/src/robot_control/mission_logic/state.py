from dataclasses import dataclass, field
from typing import Dict

@dataclass
class ZoneState:
    yagura: int = 0
    rings: int = 0

@dataclass
class GameState:
    # Robot State
    robot_x: float = 0.3
    robot_y: float = 0.5
    held_yagura: int = 0
    held_rings: int = 0
    
    # Field State
    # Zones 1-5 (Jintori)
    zones: Dict[int, ZoneState] = field(default_factory=dict)
    
    # Honmaru
    honmaru_rings: int = 0
    
    def __post_init__(self):
        # Initialize zones 1-5
        for i in range(1, 6):
            if i not in self.zones:
                self.zones[i] = ZoneState()
