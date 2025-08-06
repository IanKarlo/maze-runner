import os
import ydlidar
import time
from numpy import deg2rad, rad2deg
import matplotlib.pyplot as plt


if __name__ == "__main__":
    #inicializando e configurando os dados do ydlidar
    my_laser= ydlidar.LaserScan()
    ydlidar.os_init()
    ports = ydlidar.lidarPortList()
    port = "/dev/ydlidar"
    for key, value in ports.items():
        port = value
        print(port)
    laser = ydlidar.CYdLidar()
    laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 230400)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10)
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 9)
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)
    laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
    laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
    laser.setlidaropt(ydlidar.LidarPropMaxRange, 16.0)
    laser.setlidaropt(ydlidar.LidarPropMinRange, 0.12)
    laser.setlidaropt(ydlidar.LidarPropInverted, True)
    laser.setlidaropt(ydlidar.LidarPropIntenstiy, False)
    ret = laser.initialize()

    if ret:
        ret = laser.turnOn()
        scan = ydlidar.LaserScan()
        count = 2
        points = []
        while ret and ydlidar.os_isOk() and count > 0:
            count -= 1
            r = laser.doProcessSimple(scan);                                                                                                                                                                                                                                                                                                                                                                                                                                                  
            if r:
                print("Scan received [ " , scan.stamp , " ]: " , scan.points.size() ," ranges is [ " ,1.0/(scan.config.scan_time+0.0005), " ] Hz")
                points.append(list(map(lambda x: (x.angle, x.range), scan.points)))
            else:
                print("Failed to get Lidar Data ")
        print(points)
        if points:
            last_scan = points[-1]  # Pegando o último conjunto de pontos

            # Separando ângulos e distâncias
            angles_deg, distances = zip(*last_scan)

            # Convertendo ângulos para radianos
            angles_rad = [deg2rad(a) for a in angles_deg]

            # Criando figura polar
            fig = plt.figure(figsize=(8, 8))
            ax = fig.add_subplot(111, projection='polar')
            ax.set_theta_zero_location('top')
            ax.set_theta_direction(-1)

            # Plotando os pontos
            ax.scatter(angles_rad, distances, s=10, c='blue', alpha=0.75)

            # Salvando a imagem
            plt.title("Scan LIDAR - Formato Radar")
            plt.savefig("lidar_scan.png")
            print("Imagem salva como lidar_scan.png")
        laser.turnOff()
    laser.disconnecting()
