from robot_msgs.msg import HandMessage, RingMechanism, WheelMessage, YaguraMechanism


class HandsController:
    """
    各ハンドの制御クラス
    """

    def __init__(self):
        self.yagura_1_pos = YaguraMechanism.POS_STOPPED
        self.yagura_1_state = YaguraMechanism.STATE_STOPPED
        self.yagura_2_pos = YaguraMechanism.POS_STOPPED
        self.yagura_2_state = YaguraMechanism.STATE_STOPPED

        self.ring_1_pos = RingMechanism.POS_STOPPED
        self.ring_1_state = RingMechanism.STATE_STOPPED
        self.ring_2_pos = RingMechanism.POS_STOPPED
        self.ring_2_state = RingMechanism.STATE_STOPPED
