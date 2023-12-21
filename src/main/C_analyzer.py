import cProfile, pstats, io
from AsteroidGame import runAsteroidGame
import time as time

def profile_run_asteroid_game():
    pr = cProfile.Profile()
    pr.enable()

    # Running the function to be profiled
    
    runAsteroidGame()
    
    pr.disable()
    s = io.StringIO()
    sortby = 'cumulative'
    ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
    ps.print_stats()
    return s.getvalue()

profile_results = profile_run_asteroid_game()
profiling_script_path = 'C:\\Users\\aslan\\OneDrive\\Dokument\\Maskinteknik år 5\\Omnibot Project\\group-f\\src\\main\\results.txt'
with open(profiling_script_path, 'w') as file:
    file.write(profile_results)


