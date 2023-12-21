
from Omnibot import omnibot
from omnibot.tcp import Connection
from time import time,sleep

# Robot connection parameters
HOST = "130.235.83.171"
PORT = 9006

DT = 0.3

def test_pid():
    with Connection(HOST, PORT) as bot:
        bot = omnibot(bot,debugMode=True)
        directions = [0,90,180,270]
        timeZero = time()


        while True:
            t0 = time()
            elapsedTime = t0 - timeZero
            if elapsedTime < 1:
                bot.move_in_dir(directions[0])
            elif elapsedTime > 1 and elapsedTime < 2:
                bot.move_in_dir(directions[1])
            elif elapsedTime > 2 and elapsedTime < 3:
                bot.move_in_dir(directions[2])
            elif elapsedTime > 3 and elapsedTime < 4:
                bot.move_in_dir(directions[3])
            else:
                bot.stop_robot()
                break
            
                
            t1 = time()
            
            if DT - (t1 - t0) > 0:
                sleep(DT - (t1 - t0))
        
    