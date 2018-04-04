# -----------------------------------------------------------------------------
# Hardware config.
REAL_TIME_DT = 1.0/25.0

# Maximum velocity for each axis in millimeter per minute.
MAX_VELOCITY_MM_PER_MIN_X = 24000
MAX_VELOCITY_MM_PER_MIN_Y = 12000
MAX_VELOCITY_MM_PER_MIN_Z = 600
MAX_VELOCITY_MM_PER_MIN_E = 1500
MIN_VELOCITY_MM_PER_MIN = 1

# Workplace physical size.
TABLE_SIZE_X_MM = 200
TABLE_SIZE_Y_MM = 200
TABLE_SIZE_Z_MM = 220

# Mixed settings.
TIP_MAX_ACCELERATION_MM_PER_S2 = 3000  # for all axis, mm per sec^2
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

