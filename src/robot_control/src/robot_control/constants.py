import math

M3508_MAX_RPM = 4000  # モータの最大回転数 [rev/min]
WHEEL_RADIUS = 0.04925  # タイヤの半径 [m]
MAX_SPEED_MPS = WHEEL_RADIUS * 2 * math.pi * (M3508_MAX_RPM / 60)  # 最大速度 [m/s]

L_X = 0.1725  # ロボットの中心からタイヤの中心まで（前後方向） [m]
L_Y = 0.2425  # ロボットの中心からタイヤの中心まで（左右方向） [m]
GEAR_RATIO = 19.20320855614973  # モータのギア比
