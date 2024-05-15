

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command 
import time, sys
from pymavlink import mavutil

takeoff_altitude= 2.5 



uav = connect('udp:192.168.1.78:14550', wait_ready=True) 



def takeoff(altitude):
    while uav.is_armable is not True:
        print("UAV is not in armorable condition!")
        time.sleep(1)

    print("UAV can armorable..")

    uav.mode= VehicleMode("GUIDED")
    

    uav.armed= True

    while uav.armed is not True:
        print("UAV is being arm...")
        time.sleep(0.5)

    print("UAV was armed.")

    uav.simple_takeoff(altitude)
    
    while uav.location.global_relative_frame.alt < altitude*0.9 :
        print("UAV rises to target.")
        time.sleep(1)

def mission():
    global myCommand
    myCommand= uav.commands

    myCommand.clear()
    time.sleep(1)

    myCommand.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 2.5)) 
   

    myCommand.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0))

    myCommand.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    
    retry_count = 3
    while retry_count > 0:
        try:
            myCommand.upload()
            print("Commands loading...")
            break
        except Exception as e:
            print("Command load error:", str(e))
            print("It will be tried again...")
            retry_count -= 1
            time.sleep(1)  
    else:
        print("Komut yükleme başarısız oldu. Güvenlik önlemi olarak drone'u güvenli bir şekilde inmeye veya RTL konumuna geri döndürülüyor.")
        
        try:
            uav.mode = VehicleMode("RTL")  
            sys.exit()  
        except Exception as e:
            print("Error applying a security measure:", str(e))


takeoff(takeoff_altitude)

mission()

myCommand.next= 0

uav.mode= VehicleMode("AUTO")

while True:
    next_waypoint= myCommand.next
    print(f'Next command {next_waypoint}')
    time.sleep(1)

    if next_waypoint is 2: 
        print("Mission done.")
        break

uav.close()
print("Out of the loop.")




