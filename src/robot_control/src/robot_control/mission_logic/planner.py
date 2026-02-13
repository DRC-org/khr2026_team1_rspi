from .state import GameState
from .rules import RuleManager
try:
    from ortools.sat.python import cp_model
except ImportError:
    cp_model = None

class ActionPlanner:
    def __init__(self):
        self.rules = RuleManager()
        if cp_model is None:
            raise ImportError("OR-Tools required")
            
        # Define base spots
        self.base_spots = {
            0: {"id": 0, "name": "CurrentPos", "type": "start", "action_time": 0},
            1: {"id": 1, "name": "Jintori_1", "x": 3.15, "y": 2.0, "type": "target", "action_time": 10, "req_y": 1, "req_r": 2},
            2: {"id": 2, "name": "Jintori_2", "x": 3.15, "y": 3.2, "type": "target", "action_time": 10, "req_y": 1, "req_r": 2},
            3: {"id": 3, "name": "Jintori_3", "x": 3.15, "y": 4.4, "type": "target", "action_time": 10, "req_y": 1, "req_r": 2},
            5: {"id": 5, "name": "Honmaru", "x": 3.25, "y": 1.40, "type": "honmaru", "action_time": 5, "req_y": 0, "req_r": 1},
            101: {"id": 101, "name": "Supply_Y", "x": 2.96, "y": 0.6, "type": "supply", "action_time": 5, "gain_y": 2, "gain_r": 0},
            201: {"id": 201, "name": "Supply_R", "x": 0.7, "y": 6.2, "type": "supply", "action_time": 5, "gain_y": 0, "gain_r": 4},
        }

    def plan_mission(self, current_state: GameState):
        model = cp_model.CpModel()
        
        # 1. Expand Graph (Multiple supply visits)
        # Create virtual nodes for supplies to allow repeated visits
        nodes = {}
        nodes[0] = self.base_spots[0]
        nodes[0]['x'] = current_state.robot_x
        nodes[0]['y'] = current_state.robot_y
        
        # Add targets that are not yet captured
        for i in [1, 2, 3]:
            if current_state.zones[i].yagura == 0:
                nodes[i] = self.base_spots[i]
        
        # Add Honmaru
        nodes[5] = self.base_spots[5]
        
        # Add virtual supplies (e.g., up to 3 visits each)
        for i in range(3):
            nodes[1000 + i] = {**self.base_spots[101], "id": 1000 + i}
            nodes[2000 + i] = {**self.base_spots[201], "id": 2000 + i}
            
        ids = list(nodes.keys())
        n = len(ids)
        
        # 2. Variables
        # x[i,j] = 1 if robot moves from i to j
        x = {}
        for i in ids:
            for j in ids:
                x[i, j] = model.NewBoolVar(f'x_{i}_{j}')
        
        visit = {i: model.NewBoolVar(f'v_{i}') for i in ids}
        order = {i: model.NewIntVar(0, n, f'o_{i}') for i in ids}
        y_hold = {i: model.NewIntVar(0, self.rules.MAX_YAGURA, f'y_{i}') for i in ids}
        r_hold = {i: model.NewIntVar(0, self.rules.MAX_RINGS, f'r_{i}') for i in ids}

        # 3. Constraints
        model.Add(visit[0] == 1)
        model.Add(order[0] == 0)
        model.Add(y_hold[0] == current_state.held_yagura)
        model.Add(r_hold[0] == current_state.held_rings)
        
        # Circuit logic
        for i in ids:
            model.Add(sum(x[i, j] for j in ids if i != j) == visit[i])
            model.Add(sum(x[j, i] for j in ids if i != j) == visit[i])
            model.Add(x[i, i] == 0)

        for i in ids:
            for j in ids:
                if i == j or j == 0: continue # Cannot return to start
                
                # Subtour elimination & Inventory
                model.Add(order[j] >= order[i] + 1).OnlyEnforceIf(x[i, j])
                
                # Resource check
                model.Add(y_hold[i] >= nodes[j].get('req_y', 0)).OnlyEnforceIf(x[i, j])
                model.Add(r_hold[i] >= nodes[j].get('req_r', 0)).OnlyEnforceIf(x[i, j])
                
                # Update
                model.Add(y_hold[j] == y_hold[i] - nodes[j].get('req_y', 0) + nodes[j].get('gain_y', 0)).OnlyEnforceIf(x[i, j])
                model.Add(r_hold[j] == r_hold[i] - nodes[j].get('req_r', 0) + nodes[j].get('gain_r', 0)).OnlyEnforceIf(x[i, j])

        # V-Goal logic
        ote_targets = [i for i in ids if nodes[i]['type'] == 'target']
        ote_achieved = model.NewBoolVar('ote_achieved')
        # Simple count: if visited enough targets
        model.Add(sum(visit[i] for i in ote_targets) >= 3).OnlyEnforceIf(ote_achieved)
        model.Add(visit[5] == 1).OnlyEnforceIf(ote_achieved)
        # Honmaru must be after targets
        for i in ote_targets:
            # If both target i and honmaru 5 are visited, 5 must be after i
            # Use a list of conditions for OnlyEnforceIf
            model.Add(order[5] > order[i]).OnlyEnforceIf([visit[i], visit[5]])

        # 4. Objective: Maximize score then minimize time/distance
        # For simplicity, focus on visiting 5 (Honmaru)
        model.Maximize(visit[5] * 1000 + sum(visit[i] for i in ote_targets) * 100 - sum(order[i] for i in ids))

        # Solve
        solver = cp_model.CpSolver()
        solver.parameters.max_time_in_seconds = 2.0
        status = solver.Solve(model)
        
        if status in [cp_model.OPTIMAL, cp_model.FEASIBLE]:
            path = []
            curr = 0
            while True:
                next_node = None
                for j in ids:
                    if j != curr and solver.Value(x[curr, j]):
                        next_node = j
                        break
                if next_node is None or next_node == 0: break
                path.append(nodes[next_node])
                curr = next_node
                if curr == 5: break
            return path
            
        return []

