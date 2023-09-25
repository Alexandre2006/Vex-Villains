# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       thinkalex                                                    #
# 	Created:      9/20/2023, 4:48:58 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *


# Brain should be defined by default
brain=Brain()

# Setup Motors
lmg = MotorGroup(Motor(Ports.PORT2, GearSetting.RATIO_18_1), Motor(Ports.PORT10, GearSetting.RATIO_18_1))
rmg = MotorGroup(Motor(Ports.PORT1, GearSetting.RATIO_18_1, True), Motor(Ports.PORT8, GearSetting.RATIO_18_1, True))

# Setup Controller
controller = Controller()

# Check axis values of controller and display them on the brain screen (all 4)
def joystick_drive():
    while True:
        hr_axis = controller.axis1.position() # Horizantal Right Joystick (percentage -100 to 100) 20
        vr_axis = controller.axis2.position() # Vertical Right Joystick (percentage -100 to 100) 100
        vl_axis = controller.axis3.position() # Vertical Left Joystick (percentage -100 to 100)
        hl_axis = controller.axis4.position() # Horizantal Left Joystick (percentage -100 to 100)
    
        # Calculate rotational power for left and right motors
        max = abs(hr_axis) + abs(vr_axis)
        if max < 100:
            max = 100
        
        # Calculate power for left and right motors
        left = vr_axis + hr_axis
        right = vr_axis - hr_axis

        print(f"Left Motor: {left}% | Right Motor: {right}%")

        # Set power for left and right motors
        lmg.spin(FORWARD, left/max*100, PERCENT)
        rmg.spin(FORWARD, right/max*100, PERCENT)

        

joystick_drive()