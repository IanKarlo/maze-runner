import os
import ydlidar
import time
import sys
import numpy as np
import serial
import csv

BAUDRATE = 230400

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

# Lista portas e escolhe a primeira encontrada
ports = ydlidar.lidarPortList()
port = "/dev/ydlidar"
for key, value in ports.items():
    port = value
print(f"Port {port}")
# Limpa o buffer serial antes de iniciar
limpar_buffer_serial(port, BAUDRATE)

# Configuração do LIDAR
laser = ydlidar.CYdLidar()
laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, BAUDRATE)
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
                
                print("Angles:", angle)
                print("Range:", ran)

                # Caminho dos arquivos
                arquivo_csv = "dados_lidar.csv"

                # Salva em CSV
                with open(arquivo_csv, mode='w', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(["Angle", "Range"])
                    for a, r in zip(angle, ran):
                        writer.writerow([a, r])

                print(f"Dados salvos em: {arquivo_csv}")

except Exception as e:
    print(f"Ocorreu um erro: {e}")
finally:
    # Encerra e desconecta o LIDAR com segurança
    laser.turnOff()
    time.sleep(1)
    laser.disconnecting()
    time.sleep(2)