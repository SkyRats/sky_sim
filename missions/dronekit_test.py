# Importar a biblioteca DroneKit
from dronekit import connect, VehicleMode
import time

# Conectar ao drone através de uma porta serial ou rede (por exemplo, UDP)
connection_string = '127.0.0.1:14550'
vehicle = connect(connection_string)

# Definir a função para imprimir informações de telemetria
def imprimir_telemetria():
    
    print("\nTELEMETRIA")

    # Imprimir informações de posição GPS
    print("Posição GPS: Lat = {0}, Lon = {1}, Alt = {2}".format(
        vehicle.location.global_frame.lat,
        vehicle.location.global_frame.lon,
        vehicle.location.global_frame.alt
    ))

    # Imprimir informações de atitude
    print("Atitude: Roll = {0}, Pitch = {1}, Yaw = {2}".format(
        vehicle.attitude.roll,
        vehicle.attitude.pitch,
        vehicle.attitude.yaw
    ))

    # Imprimir informações de velocidade
    print("Velocidade: Vx = {0}, Vy = {1}, Vz = {2}".format(
        vehicle.velocity[0],
        vehicle.velocity[1],
        vehicle.velocity[2]
    ))

    # Imprimir altura relativa do drone
    print(f"Altitude relativa (m): {vehicle.location.global_relative_frame.alt}\n")

    # Aguardar um curto intervalo de tempo antes de imprimir novamente
    time.sleep(1)


def arm_and_takeoff(altitude):
    print("Armando os motores...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.is_armable:
        print("Esperando para armar...")
        time.sleep(1)

    while not vehicle.armed:
        print("Esperando para armar...")
        time.sleep(1)

    print("Decolando...")
    vehicle.simple_takeoff(altitude)

    while True:
        if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
            print("Altitude de 10 metros alcançada.")
            break
        time.sleep(1)

def land():
    print("Iniciando o procedimento de pouso...")
    vehicle.mode = VehicleMode("LAND")

    while not vehicle.mode.name == "LAND":
        time.sleep(1)

    print("Pousando...")
    while vehicle.location.global_relative_frame.alt > 0.1:
        time.sleep(1)

    print("Drone pousou com sucesso.")

# Função para retornar e pousar
def retornar_e_pousar():
    vehicle.mode = VehicleMode("RTL")  # Definir o modo de voo como "Return to Launch"
    print("Retornando e pousando...")

def print_status():
    #-- Check vehicle status
    print("\nSTATUS")
    print(f"Mode: {vehicle.mode.name}")
    print(" Global Location: %s" % vehicle.location.global_frame)
    print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    print(" Local Location: %s" % vehicle.location.local_frame)
    print(" Gimbal status: %s" % vehicle.gimbal)
    print(" EKF OK?: %s" % vehicle.ekf_ok)
    print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
    print(" Is Armable?: %s" % vehicle.is_armable)
    print(" System status: %s" % vehicle.system_status.state)
    print(" Armed: %s" % vehicle.armed)    # settable

# Programa principal
try:
    print_status()
    imprimir_telemetria()
    arm_and_takeoff(10)
    time.sleep(5)  # Esperar por 5 segundos antes de pousar
    imprimir_telemetria()
    land()
    imprimir_telemetria()

except KeyboardInterrupt:
    print("Aplicativo interrompido pelo usuário.")

finally:
    # Desarmar os motores e fechar a conexão com o drone
    vehicle.armed = False
    vehicle.close()


