# -----------------------------------------------------------------------------
# Hardware config.
REAL_TIME_DT = 1.0/25.0

DELTA_SMALL_L = 0.04
DELTA_BIG_L = 0.02
DELTA_WB = 0.02
DELTA_UP = 0.00922
DELTA_Z_OFFSET = -0.04

MOTOR0_OFFSET_RAD = -0.16
MOTOR1_OFFSET_RAD = -0.16
MOTOR2_OFFSET_RAD = -0.16

# Maximum velocity for each axis in millimeter per minute.
MAX_VELOCITY_MM_PER_MIN_X = 24000
MAX_VELOCITY_MM_PER_MIN_Y = 12000
MAX_VELOCITY_MM_PER_MIN_Z = 600
MAX_VELOCITY_MM_PER_MIN_E = 1500
MIN_VELOCITY_MM_PER_MIN = 1

# Mixed settings.
TIP_MAX_ACCELERATION_MM_PER_S2 = 30  # for all axis, mm per sec^2
SPINDLE_MAX_RPM = 10000

# -----------------------------------------------------------------------------
# Pins configuration.

SPINDLE_PWM_PIN = 4
FAN_PIN = 27

# -----------------------------------------------------------------------------
#  Behavior config

# Run command immediately after receiving and stream new pulses, otherwise
# buffer will be prepared firstly and then command will run.
# Before enabling this feature, please make sure that board performance is
# enough for streaming pulses(faster then real time).
INSTANT_RUN = False

# If this parameter is False, error will be raised on command with velocity
# more than maximum velocity specified here. If this parameter is True,
# velocity would be decreased(proportional for all axises) to fit the maximum
# velocity.
AUTO_VELOCITY_ADJUSTMENT = True

