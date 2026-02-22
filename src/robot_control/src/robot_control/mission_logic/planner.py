import math
from .state import GameState
from .rules import RuleManager

class ActionPlanner:
    def __init__(self):
        self.rules = RuleManager()
            
        # Define base spots (Flipped X version - consistent with Spawn X=2.7)
        self.base_spots = {
            1: {"id": 1, "name": "Jintori_1", "x": 0.35, "y": 2.0, "type": "target", "action_time": 10, "req_y": 1, "req_r": 2},
            2: {"id": 2, "name": "Jintori_2", "x": 0.35, "y": 3.2, "type": "target", "action_time": 10, "req_y": 1, "req_r": 2},
            3: {"id": 3, "name": "Jintori_3", "x": 0.35, "y": 4.4, "type": "target", "action_time": 10, "req_y": 1, "req_r": 2},
            5: {"id": 5, "name": "Honmaru", "x": 0.25, "y": 1.40, "type": "honmaru", "action_time": 5, "req_y": 0, "req_r": 1},
            101: {"id": 101, "name": "Supply_Y", "x": 0.54, "y": 0.6, "type": "supply", "action_time": 5, "gain_y": 2, "gain_r": 0},
            201: {"id": 201, "name": "Supply_R", "x": 2.8, "y": 6.2, "type": "supply", "action_time": 5, "gain_y": 0, "gain_r": 4},
        }

    def _dist(self, x1, y1, x2, y2):
        return math.hypot(x1 - x2, y1 - y2)

    def plan_mission(self, current_state: GameState):
        """Greedy logic based planner (Lightweight)"""
        path = []
        cur_x = current_state.robot_x
        cur_y = current_state.robot_y
        held_y = current_state.held_yagura
        held_r = current_state.held_rings

        # 1. Identify remaining targets
        targets = []
        for i in [1, 2, 3]:
            if i in current_state.zones and current_state.zones[i].yagura == 0:
                targets.append(self.base_spots[i])

        # 2. Logic loop
        while targets:
            targets.sort(key=lambda t: self._dist(cur_x, cur_y, t['x'], t['y']))
            next_t = targets[0]

            if held_y >= next_t['req_y'] and held_r >= next_t['req_r']:
                path.append(next_t)
                cur_x, cur_y = next_t['x'], next_t['y']
                held_y -= next_t['req_y']
                held_r -= next_t['req_r']
                targets.pop(0)
            else:
                if held_y < next_t['req_y']: supply = self.base_spots[101]
                else: supply = self.base_spots[201]
                
                path.append(supply)
                cur_x, cur_y = supply['x'], supply['y']
                held_y += supply.get('gain_y', 0)
                held_r += supply.get('gain_r', 0)
            if len(path) > 8: break

        if not targets:
            honmaru = self.base_spots[5]
            if held_y >= honmaru['req_y'] and held_r >= honmaru['req_r']:
                path.append(honmaru)
            else:
                path.append(self.base_spots[201])
                path.append(honmaru)
        return path
