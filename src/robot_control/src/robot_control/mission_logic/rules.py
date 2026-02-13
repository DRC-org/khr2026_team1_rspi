from .state import GameState

class RuleManager:
    def __init__(self, is_auto=True):
        self.is_auto = is_auto
        
        # Hardware Constraints
        self.MAX_YAGURA = 2
        self.MAX_RINGS = 4
        
        # Rule Constraints
        self.AREA_2_LIMIT = 5 if is_auto else 3
        
    def can_pick_yagura(self, state: GameState, amount: int) -> bool:
        return (state.held_yagura + amount) <= self.MAX_YAGURA

    def can_pick_ring(self, state: GameState, amount: int) -> bool:
        return (state.held_rings + amount) <= self.MAX_RINGS

    def can_enter_area_2(self, state: GameState) -> bool:
        """Area 2 (Yagura/Honmaru) entry restriction"""
        return state.held_rings <= self.AREA_2_LIMIT

    def is_ote(self, state: GameState) -> bool:
        """Check Ote condition"""
        ote_zones = 0
        for zone in state.zones.values():
            if self.is_auto:
                # Auto: Yagura + 2 Rings
                if zone.yagura >= 1 and zone.rings >= 2:
                    ote_zones += 1
            else:
                # Manual: Simple Jintori count (simplified)
                if zone.yagura >= 1 and zone.rings > 0:
                    ote_zones += 1
        
        return ote_zones >= 3

    def is_v_goal(self, state: GameState) -> bool:
        return self.is_ote(state) and state.honmaru_rings > 0
