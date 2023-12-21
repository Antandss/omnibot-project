
from TestPID import test_pid
from AsteroidGame import runAsteroidGame



def main_mode():
    print("Main Running Mode")

def asteroid_mode():
    print("Running Asteroid mode.")
    runAsteroidGame()
    1
    
def pid_test():
    print("Running PID test mode.")
    test_pid()

def transformation_check():
    #camera_transformation()
    print("")
    
def print_menu():
    print("1: Asteroid Mode CURRENTLY BROKEN")
    print("2: Test Controller Mode")
    print("3: Transformation MODE")
    print("0: Exit")

def get_choice():
    return input("Enter your choice: ")

menu_options = {
    '1': asteroid_mode,
    '2': pid_test,
    
}

def main():
    runAsteroidGame()
    
    while True:
        print_menu()

        choice = get_choice()

        if choice == '0':
            break
        elif choice in menu_options:
            menu_options[choice]()
        else:
            print("Invalid choice. Please try again.")
            

if __name__ == "__main__":
    main()
    



    


    

 







