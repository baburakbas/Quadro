import time, sys
from pymavlink import mavutil

takeoff_altitude= 2.5

uav= connect("/dev/ttyAMA0", baud=921600)
#uav =connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
#uav  =connect('udp:192.168.4.2:14550', wait_ready=True)


def takeoff(altitude):

#GUDIDED_NOGPS, GUIDED,ALT_HOLD,STABILIZED
    uav.armed= True

    while uav.is_armable is  True:
        print("UAV is not in armorable condition!")
        print(uav.is_armable)
        time.sleep(1)

    print("UAV can armorable..")

    uav.mode= VehicleMode("GUIDED")
    while uav.mode!="GUIDED":
        print("wait for guided mode yeey")
        time.sleep(1)

    uav.armed= True

    while uav.armed is not True:
        print("UAV is being arm...")
        time.sleep(0.5)

    print("UAV was armed.")

    uav.simple_takeoff(altitude)
    while True:
        current_altitude = uav.location.global_relative_frame.alt
        if current_altitude is not None and current_altitude >= altitu>
            print("UAV has reached the target altitude.")
            break
        print("UAV rises to target.")
        time.sleep(1)
def mission():
    global myCommand
    myCommand= uav.commands

    myCommand.clear()
    time.sleep(1)

    myCommand.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RE>


    myCommand.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RE>

    myCommand.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RE>

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
        print("Komut yükleme başarısız oldu. Güvenlik önlemi olarak dr>

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

