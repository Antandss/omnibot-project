from ControllerCVXPY_Gen2 import ControllerMPC

if __name__ == "__main__":
    # Create an instance of the MPCController
    ControllerMPC = ControllerMPC()

    current_states = [0.0, 0.0, 0.0]
    desired_setpoints = [0.5, 0.5, 0.0]

    u = ControllerMPC.get_control_action(current_states,desired_setpoints)

    # Print the obtained control signals
    print('control signals' + str(u))