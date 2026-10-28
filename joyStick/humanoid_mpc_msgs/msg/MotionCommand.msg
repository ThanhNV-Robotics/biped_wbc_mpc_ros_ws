# Motion command of the WBC controller (keyboard / joystick), published once per key press.
# Numbers = the commands of the WBC plan (biped_controllers/WBC_IMPLEMENTATION_PLAN.md).
uint8 HOMING = 1        # ramp all the joints to 0, both feet on the ground
uint8 INIT_WALKING = 2  # lower the base to the initial walking posture (bend the knees)
uint8 WALKING = 3       # sent repeatedly (20 Hz) while the walk key is held: the robot steps;
                        # when the messages stop it finishes the step and comes to rest

uint8 command
