from .mission_logic.state import GameState

class ScoringManager:
    def __init__(self, is_auto=True):
        self.is_auto = is_auto
        self.entered_yagura_zone = False
        self.zones = {}
        self.rings_in_honmaru = 0
        
    def get_total_score(self):
        score = 0
        if self.entered_yagura_zone: score += 20 if self.is_auto else 10
        jintori_count = 0
        ote_eligible_zones = 0
        for zid, state in self.zones.items():
            y_count = state.get('yagura_count', 0)
            r_in_y = state.get('rings_in_yagura', 0)
            r_floor = state.get('rings_on_floor', 0)
            score += y_count * (10 if self.is_auto else 5)
            score += r_floor * (4 if self.is_auto else 2)
            score += r_in_y * (20 if self.is_auto else 10)
            if r_in_y > 0:
                score += 30
                jintori_count += 1
            if self.is_auto and r_in_y >= 2: ote_eligible_zones += 1
        if self.is_ote(jintori_count, ote_eligible_zones): score += 100 if self.is_auto else 50
        score += self.rings_in_honmaru * (50 if self.is_auto else 25)
        return score

    def report_yagura_zone_entry(self): self.entered_yagura_zone = True
    def update_zone_state(self, zone_id, yagura_count, rings_in_yagura, rings_on_floor):
        self.zones[zone_id] = {'yagura_count': yagura_count, 'rings_in_yagura': rings_in_yagura, 'rings_on_floor': rings_on_floor}
    def update_honmaru_state(self, rings_in_honmaru): self.rings_in_honmaru = rings_in_honmaru
    def is_ote(self, jintori_count=None, ote_eligible_zones=None):
        if jintori_count is None:
            jintori_count = 0
            ote_eligible_zones = 0
            for s in self.zones.values():
                if s.get('rings_in_yagura', 0) > 0: jintori_count += 1
                if s.get('rings_in_yagura', 0) >= 2: ote_eligible_zones += 1
        return ote_eligible_zones >= 3 if self.is_auto else jintori_count >= 3
    def is_v_goal(self): return self.is_ote() and self.rings_in_honmaru > 0
