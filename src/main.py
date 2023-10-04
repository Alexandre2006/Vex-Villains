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
lmg = MotorGroup(Motor(Ports.PORT19, GearSetting.RATIO_18_1), Motor(Ports.PORT20, GearSetting.RATIO_18_1))
rmg = MotorGroup(Motor(Ports.PORT14, GearSetting.RATIO_18_1, True), Motor(Ports.PORT15, GearSetting.RATIO_18_1, True))

# Setup Controller
controller = Controller()

class DriveMode:
    LEFT_JOYSTICK_ONLY = 0
    RIGHT_JOYSTICK_ONLY = 1
    DRONE = 2
    TANK = 3

# Convert to string
def DriveModeToString(mode):
    if mode == DriveMode.LEFT_JOYSTICK_ONLY:
        return "Left Joystick"
    elif mode == DriveMode.RIGHT_JOYSTICK_ONLY:
        return "Right Joystick"
    elif mode == DriveMode.DRONE:
        return "Drone"
    elif mode == DriveMode.TANK:
        return "Tank"

# Set drive mode
drive_mode = DriveMode.TANK
turn_multiplier = 0.5

# Check axis values of controller and display them on the brain screen (all 4)
def joystick_drive():
    while True:
        if (drive_mode != DriveMode.TANK):
            hr_axis = controller.axis1.position() # Horizantal Right Joystick (percentage -100 to 100)
            vr_axis = controller.axis2.position() # Vertical Right Joystick (percentage -100 to 100)
            vl_axis = controller.axis3.position() # Vertical Left Joystick (percentage -100 to 100)
            hl_axis = controller.axis4.position() # Horizantal Left Joystick (percentage -100 to 100)

            # Adjust turning
            if drive_mode == DriveMode.RIGHT_JOYSTICK_ONLY:
                hr_axis = hr_axis * turn_multiplier
            else:
                hl_axis = hl_axis * turn_multiplier
        
            # Calculate rotational power for left and right motors
            max = abs(hl_axis) + abs(vr_axis)
            if max < 100:
                max = 100

            # Calculate power for left and right motors
            left = 0
            right = 0
            if drive_mode == DriveMode.LEFT_JOYSTICK_ONLY:
                print("1")
                left = vl_axis + hl_axis
                right = vl_axis - hl_axis
            elif drive_mode == DriveMode.RIGHT_JOYSTICK_ONLY:
                print("1")
                left = vr_axis + hr_axis
                right = vr_axis - hr_axis
            elif drive_mode == DriveMode.DRONE:
                print("1")
                left = vr_axis + hl_axis
                right = vr_axis - hl_axis
            
            print("Left: " + str(hl_axis) + " Right: " + str(hr_axis))
            
            # Set power for left and right motors
            lmg.spin(FORWARD, left/max*100, PERCENT)
            rmg.spin(FORWARD, right/max*100, PERCENT)
        else:
            lmg.spin(FORWARD, controller.axis3.position(), PERCENT)
            rmg.spin(FORWARD, controller.axis2.position(), PERCENT)

def toggleDriveModeUp():
    global drive_mode
    drive_mode = (drive_mode + 1) % 4
    renderInfo()

def toggleDriveModeDown():
    global drive_mode
    drive_mode = (drive_mode - 1) % 4
    renderInfo()

def increaseTurnMultiplier():
    global turn_multiplier
    if turn_multiplier < 1:
        turn_multiplier = turn_multiplier + 0.01
        renderInfo()

def decreaseTurnMultiplier():
    global turn_multiplier
    if turn_multiplier > 0:
        turn_multiplier = turn_multiplier - 0.01
        renderInfo()

def renderInfo():
    controller.screen.clear_screen()
    controller.screen.set_cursor(0, 0)
    controller.screen.print("Drive Mode: " + str(DriveModeToString(drive_mode)))
    if drive_mode != DriveMode.TANK:
        controller.screen.new_line()
        controller.screen.print("Turn Multiplier: " + str(turn_multiplier))

# Toggle drive mode (when up or down button is pressed)
controller.buttonUp.pressed(toggleDriveModeUp)
controller.buttonDown.pressed(toggleDriveModeDown)

# Increase or decrease turn multiplier (when left or right button is pressed)
controller.buttonLeft.pressed(decreaseTurnMultiplier)
controller.buttonRight.pressed(increaseTurnMultiplier)

renderInfo()  
joystick_drive()