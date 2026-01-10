import math

M3508_MAX_RPM = 400  # モータの最大回転数 [rev/min]
WHEEL_RADIUS = 0.04925  # タイヤの半径 [m]
MAX_SPEED_MPS = WHEEL_RADIUS * 2 * math.pi * (M3508_MAX_RPM / 60)  # 最大速度 [m/s]
