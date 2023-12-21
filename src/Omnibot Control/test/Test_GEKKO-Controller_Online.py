from omnibot.tcp import Connection
from time import time, sleep
from MPC_controller_GEKKO import MPCController

# Insert suitable IP-adress
HOST = "130.235.83.171"
PORT = 9001

with Connection(HOST, PORT) as bot:

        # Record start time
        t0 = time()

        initPos = [bot.get_x(), bot.get_y(), bot.get_theta()]
        mpc_controller = MPCController(initPos)
        desired_setpoints = [0.5, 0.5, 0.0]
        mpc_controller.set_desired_setpoints(desired_setpoints)
        
        while True:
            
            current_states = [bot.get_x(), bot.get_y(), bot.get_theta()]

            optimal_controls = mpc_controller.get_control_signals(current_states, desired_setpoints)
            # set speeds
            const_mult = 10000
            bot.set_speeds([int(optimal_controls[0])*const_mult,int(optimal_controls[1])*const_mult,int(optimal_controls[2])*const_mult])
            
            # print position
            print('x:'+str(bot.get_x()))
            print('y:'+str(bot.get_y()))

            sleep(0.1)
