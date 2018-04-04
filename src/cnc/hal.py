from cnc.hal_file_exporter import *

# check if all methods that is needed is implemented
if 'init' not in locals():
    raise NotImplementedError("hal.init() not implemented")
if 'spindle_control' not in locals():
    raise NotImplementedError("hal.spindle_control() not implemented")
if 'fan_control' not in locals():
    raise NotImplementedError("hal.fan_control() not implemented")
if 'disable_steppers' not in locals():
    raise NotImplementedError("hal.disable_steppers() not implemented")
if 'move' not in locals():
    raise NotImplementedError("hal.move() not implemented")
if 'join' not in locals():
    raise NotImplementedError("hal.join() not implemented")
if 'deinit' not in locals():
    raise NotImplementedError("hal.deinit() not implemented")

