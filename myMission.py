#my mission

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command #buradaki kodu sim ortamı ı ile bağlar
import time, sys
from pymavlink import mavutil

takeoff_altitude= 2.5 #istediğşn ilk kalkış altitude için burayı değiştir


# # WI-FI CONNECTION
uav = connect('udp:192.168.1.78:14550', wait_ready=True) #IP adresi, drone'un Wi-Fi ağına bağlı olduğu IP adresini temsil eder.
# #14550 ise kullanılan port numarası
#connect, raspberry ve pixhawk arasında usb, uart ya da wi-fi ile bağlantıyı sağlar

# # UDP CONNECTION
# uav = connect('udp:0.0.0.0:14550', wait_ready=True)

#SIMULATION CONNECTION
# uav = connect('127.0.0.1:14550', wait_ready=True)
#127 olan numara gazebodaki port gibi bişi,SITL connected to the vehicle via UDP
#true ile bağlanana kadar dener

def takeoff(altitude):
    while uav.is_armable is not True:
        print("UAV is not in armorable condition!")
        time.sleep(1)

    print("UAV can armorable..")

    uav.mode= VehicleMode("GUIDED")
    
    #print(str(uav.mode)+" modu received.")

    uav.armed= True

    while uav.armed is not True:
        print("UAV is being arm...")
        time.sleep(0.5)

    print("UAV was armed.")

    uav.simple_takeoff(altitude)#istenen irtifaya çıkartırırz
    
    while uav.location.global_relative_frame.alt < altitude*0.9 :#0.9 hata payı içim
        print("UAV rises to target.")
        time.sleep(1)

def mission():
    global myCommand
    myCommand= uav.commands

    myCommand.clear()#droneda görev var ise bunu siler
    time.sleep(1)

    #TAKEOFF
    myCommand.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 2.5)) #görev ekleyebilmemize yarar
    #random, camera filan diyor tekrar bak, 
    #ardupilot mission command sayfasında detaylı komutlar var
    #0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 

    #WAYPOINT
    # myCommand.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, -35.36265286, 149.16514170, 3))#sonlarıdaki 3.sıfır göreve gittikten sonra bekleme süresi
    # myCommand.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, -35.36318559, 149.16607666, 3.5))
    #RTL
    myCommand.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0))

    #VERIFICATION
    myCommand.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    
    retry_count = 3
    while retry_count > 0:
        try:
            myCommand.upload() # komutları araca yükle
            print("Commands loading...")
            break
        except Exception as e:
            print("Command load error:", str(e))
            print("It will be tried again...")
            retry_count -= 1
            time.sleep(1)  # Tekrar deneme aralığını ayarlanabilir
    else:
        print("Komut yükleme başarısız oldu. Güvenlik önlemi olarak drone'u güvenli bir şekilde inmeye veya RTL konumuna geri döndürülüyor.")
        
        try:
            uav.mode = VehicleMode("RTL")   # RTL (Return to Launch) komutu
            sys.exit()  # Sistem sonlandırılır
        except Exception as e:
            print("Error applying a security measure:", str(e))


takeoff(takeoff_altitude)

mission()

myCommand.next= 0 # görevden sonraki komutu 0 olsun bir karışıklık olmasın diye

uav.mode= VehicleMode("AUTO") #bu tarz görev yüklenenlerde guided yerine auto mode kullanılmalıdır

while True:
    next_waypoint= myCommand.next
    print(f'Next command {next_waypoint}')
    time.sleep(1)

    if next_waypoint is 2: #veridication kısmı sadece burayı 4 diyebilmek için ekledik.
        print("Mission done.")
        break

uav.close() #udp bağlantısını kapatır
print("Out of the loop.")

#Drone’un armable (silahlanabilir) veya guided (rehberli) modlara geçiş yapabilmesi için telemetri verilerini almak ve uygulamak, DroneKit-Python kütüphanesi kullanılarak gerçekleştirilebilir.


