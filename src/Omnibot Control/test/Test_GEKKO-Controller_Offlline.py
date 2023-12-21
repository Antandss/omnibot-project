from MPC_controller_GEKKO import MPCController

if __name__ == "__main__":
    # Create an instance of the MPCController
    mpc_controller = MPCController([0,0,0])

    desired_setpoints = [0.5, 0.0, 0]
    mpc_controller.set_desired_setpoints(desired_setpoints)

    # To get updated control signals, call the get_control_signals method
    optimal_controls = mpc_controller.get_control_signals([0,0,0], [0.5, 0.0, 0])

    # Print the obtained control signals
    for i, control_signal in enumerate(optimal_controls):
        print(f"Control Signal {i + 1}: {control_signal}")

    # You can now use optimal_controls to apply the control signals to your system's actuators
    # (This part depends on your specific system setup)