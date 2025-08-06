import os
import ydlidar
import time
import sys
from matplotlib.patches import Arc
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import serial

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


RMAX = 32.0

# Inicializa a figura e o gráfico polar
fig = plt.figure()
lidar_polar = plt.subplot(polar=True)
lidar_polar.autoscale_view(True, True, True)
lidar_polar.set_rmax(RMAX)
lidar_polar.grid(True)

# Lista portas e escolhe a primeira encontrada
ports = ydlidar.lidarPortList()
port = "/dev/ydlidar"
for key, value in ports.items():
    port = value

limpar_buffer_serial(port, 230400)

# Configuração do LIDAR
laser = ydlidar.CYdLidar()
laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 230400)
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
laser.setlidaropt(ydlidar.LidarPropSampleRate, 9)
laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)

try:
    scan = ydlidar.LaserScan()
    ret = laser.initialize()
    if ret:
        ret = laser.turnOn()
        if ret:
            r = laser.doProcessSimple(scan)
            if r:
                angle = []
                ran = []
                intensity = []
                for point in scan.points:
                    angle.append(point.angle)
                    ran.append(point.range)
                    intensity.append(point.intensity)

                print(ran)
except Exception:
    pass
finally:
    laser.turnOff()
    time.sleep(1)
    laser.disconnecting()
    time.sleep(2)
    plt.close()
