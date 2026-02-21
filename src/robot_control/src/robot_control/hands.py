from robot_msgs.msg import HandMessage, RingMechanism, WheelMessage, YaguraMechanism


class HandsController:
    """
    各ハンドの制御クラス
    """

    def __init__(self):
        self.yagura_1_pos = YaguraMechanism.POS_STOPPED
        self.yagura_1_state = YaguraMechanism.STATE_CLOSE_DONE
        self.yagura_2_pos = YaguraMechanism.POS_STOPPED
        self.yagura_2_state = YaguraMechanism.STATE_CLOSE_DONE

        self.ring_1_pos = RingMechanism.POS_STOPPED
        self.ring_1_state = RingMechanism.STATE_STOPPED
        self.ring_2_pos = RingMechanism.POS_STOPPED
        self.ring_2_state = RingMechanism.STATE_STOPPED

    def set_target(self, target: str, control_type: str, action: int) -> None:
        if target == "yagura_1":
            if control_type == "pos":
                self.yagura_1_pos = action
            elif control_type == "state":
                self.yagura_1_state = action
        elif target == "yagura_2":
            if control_type == "pos":
                self.yagura_2_pos = action
            elif control_type == "state":
                self.yagura_2_state = action
        elif target == "ring_1":
            if control_type == "pos":
                self.ring_1_pos = action
            elif control_type == "state":
                self.ring_1_state = action
        elif target == "ring_2":
            if control_type == "pos":
                self.ring_2_pos = action
            elif control_type == "state":
                self.ring_2_state = action
