import json
from .state import GameState, ZoneState

class LogicManager:
    def __init__(self, state: GameState = None):
        self.state = state or GameState()
        self.is_executing_hardware = False
        self.waiting_for_feedback_id = None
        
    def process_feedback(self, msg_data: str) -> bool:
        """
        Process hardware feedback JSON.
        Returns True if an expected action was completed.
        """
        try:
            data = json.loads(msg_data)
            received_id = data.get('id')
            
            if self.is_executing_hardware and received_id == self.waiting_for_feedback_id:
                self.is_executing_hardware = False
                self.waiting_for_feedback_id = None
                return True
        except:
            pass
        return False

    def sync_state(self, score_detail_json: str):
        """Update GameState from scoring sensor data"""
        try:
            data = json.loads(score_detail_json)
            if 'zones' in data:
                for zid_str, zdata in data['zones'].items():
                    zid = int(zid_str)
                    if zid in self.state.zones:
                        self.state.zones[zid].yagura = zdata.get('y', self.state.zones[zid].yagura)
                        self.state.zones[zid].rings = zdata.get('r', self.state.zones[zid].rings)
            if 'honmaru_rings' in data:
                self.state.honmaru_rings = data['honmaru_rings']
        except:
            pass

    def prepare_action(self, action_type: str, gain_y: int = 0, gain_r: int = 0, mech_index: int = 1):
        """Set up flags for an upcoming action, supporting multiple mechanism sets"""
        self.is_executing_hardware = True
        
        if action_type == 'supply':
            if gain_y > 0:
                # 0x40 for Hand 1, 0x41 for Hand 2
                self.waiting_for_feedback_id = 0x40 if mech_index == 1 else 0x41
            elif gain_r > 0:
                # 0x4A for Hand 1, 0x4C for Hand 2
                self.waiting_for_feedback_id = 0x4A if mech_index == 1 else 0x4C
        elif action_type == 'target':
            self.waiting_for_feedback_id = 0x40 if mech_index == 1 else 0x41
        elif action_type == 'honmaru':
            self.waiting_for_feedback_id = 0x4A if mech_index == 1 else 0x4C
