from omnibot.tcp import Connection
from time import time, sleep

# Robot connection parameters
HOST = "130.235.83.171"
PORT = 9005


with Connection(HOST, PORT) as bot:
        
        # Target speed for servos
        vset = 100
        
        # Record start time
        t0 = time()

        # Go one way for 3 seconds
        while time() < t0 + 3:

            # set speeds
            bot.set_speeds([vset,vset,vset])
            
            # print position
            print('x:'+str(bot.get_x()))
            print('y:'+str(bot.get_y()))

            sleep(0.1)
