# [r,p,t,y]
# Format: [roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4]
# AUX1 is used for arming/disarming:
# - AUX1 < 1500: disarmed
# - AUX1 > 1800: armed

# Disarm command: AUX1 at 1400 (below 1500)
disarm_cmd = [1500, 1500, 1000, 1500, 1400, 1000, 1000, 1000]

# Arm command: AUX1 at 1900 (above 1800)
arm_cmd = [1500, 1500, 1000, 1500, 1900, 1000, 1000, 1000]

# Idle command: keep armed with AUX1 at 1900
idle_cmd = [1500, 1500, 1000, 1500, 1900, 1000, 1000, 1000]
