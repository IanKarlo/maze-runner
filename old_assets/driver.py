# import ydlidar
# import time
# import serial
# import os
# from math import pi

# BAUDRATE = 230400

# def scan_environment(max_dist = 4):
#     # Lista portas e escolhe a primeira encontrada
#     ydlidar.os_init()
#     ports = ydlidar.lidarPortList()
#     port = "/dev/ydlidar"
#     for _, value in ports.items():
#         port = value

#     # Configuração do LIDAR
#     laser = ydlidar.CYdLidar()
#     laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
#     laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, BAUDRATE)
#     laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
#     laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
#     laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
#     laser.setlidaropt(ydlidar.LidarPropSampleRate, 9)
#     laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)

#     angle = []
#     ran = []

#     try:
#         ret = laser.initialize()
#         if ret:
#             ret = laser.turnOn()
#             scan = ydlidar.LaserScan()
#             if ret and ydlidar.os_isOk():
#                 r = laser.doProcessSimple(scan)
#                 if r:
#                     intensity = []
#                     for point in scan.points:
#                         angle.append(point.angle)
#                         ran.append(point.range)
#                         intensity.append(point.intensity)
                    
#                     print("Angles:", angle)
#                     print("Range:", ran)

#     except Exception as e:
#         print(f"Ocorreu um erro: {e}")
#     finally:
#         # Encerra e desconecta o LIDAR com segurança
#         laser.turnOff()
#         time.sleep(1)
#         laser.disconnecting()
#         time.sleep(2)


    # transformed_angles = list(map(lambda angle: angle - (2*pi)/3, angle))
    # transformed_distances = list(map(lambda distance: distance if distance < max_dist else 0, ran))

    # return transformed_angles, transformed_distances


import os
import ydlidar
import time
from numpy import deg2rad, rad2deg
import matplotlib.pyplot as plt
import serial
import sys

def limpar_buffer_serial(porta="/dev/ttyUSB0", baudrate=115200):
    try:
        ser = serial.Serial(porta, baudrate, timeout=0.5)
        time.sleep(0.5)  # espera a porta estabilizar
        ser.reset_input_buffer()   # limpa dados recebidos
        ser.reset_output_buffer()  # limpa dados a serem enviados
        ser.close()
        print("Buffer serial limpo com sucesso.")
    except Exception as e:
        print(f"Erro ao limpar buffer: {e}")


def scan_environment():
    #inicializando e configurando os dados do ydlidar
    ydlidar.os_init()
    time.sleep(1)
    ports = ydlidar.lidarPortList()
    port = "/dev/ttyUSB0"
    for key, value in ports.items():
        port = value

    laser = ydlidar.CYdLidar()
    laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 230400)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 9)
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)
    
    limpar_buffer_serial(port, 230400)
    scan = ydlidar.LaserScan()
    time.sleep(1)
    ret = laser.initialize()


    if ret and ydlidar.os_isOk():
        ret = laser.turnOn()
        count = 2
        points = []
        while ret and count > 0:
            count -= 1
            r = laser.doProcessSimple(scan);                                                                                                                                                                                                                                                                                                                                                                                                                                                  
            if r:
                print("Scan received [ " , scan.stamp , " ]: " , scan.points.size() ," ranges is [ " ,1.0/(scan.config.scan_time+0.0005), " ] Hz")
                points.append(list(map(lambda x: (x.angle, x.range), scan.points)))
            else:
                print("Failed to get Lidar Data ")
        print(points)
        laser.turnOff()
    laser.disconnecting()

