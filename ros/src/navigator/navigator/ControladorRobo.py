import rclpy
from rclpy.node import Node
import math
import numpy as np

from project_interfaces.srv import LidarResponse, TimedMoveResponse

class EndOfPath(Exception):
    pass

largura_robo = 0.65              # Largura do robô em metros
resolucao_angular = 15            # Tamanho de cada setor (graus)
num_setores = 360 // resolucao_angular  # Total de setores
DISTANCIA_MAX = 0.5     # Limite máximo de distância em metros (50 cm)

def convert_from_points_to_lists(points):
    angles = list(map(lambda x: x.angle, points))
    distances = list(map(lambda x: x.distance, points))

    return angles, distances

def vfh(angles, distances):
    distancias_por_setor = criar_vetor_distancias(angles, distances, resolucao_angular)
    densidade = construir_histograma(distancias_por_setor)
    direcao = escolher_direcao(densidade, distancias_por_setor, largura_robo, objetivo_direcao=0, resolucao_angular=resolucao_angular)

    print(f'direcao: {direcao}')

    if direcao == None:
        raise EndOfPath()

    direcao_final = direcao - 360 if direcao > 90 else direcao
    orientation_sign = -1.0 if direcao_final < 0 else 1.0
    distancia_final = 0.2

    return math.fabs(direcao_final), orientation_sign, distancia_final

def criar_vetor_distancias(angulos_rad, medidas, resolucao_angular):
    num_setores = 360 // resolucao_angular
    distancias = np.ones(num_setores) * 10.0  # valor alto = sem obstáculo
    for ang_rad, dist in zip(angulos_rad, medidas):
        # Aplica offset de segurança
        dist_segura = max(0, dist - 0.15)  # subtrai 15 cm, sem negativos
        # dist_segura = dist
        ang_deg = math.degrees(ang_rad) % 360
        indice = int(ang_deg // resolucao_angular)
        distancias[indice] = min(distancias[indice], dist_segura)

    # Força obstáculos entre 90° e 270°
    for i, ang_deg in enumerate(np.arange(0, 360, resolucao_angular)):
        if 90 <= ang_deg < 270:
            distancias[i] = 0  # simula obstáculo muito próximo

    return distancias


# === 3. Histograma de densidade inversa ===
def construir_histograma(distancias, limiar=1.5): 
    print("distancias:", distancias)
    densidade = np.where(distancias < limiar, 1 / distancias, 0)
    print(densidade)
    return densidade

# === 4. Escolha da direção ideal considerando a largura do robô ===
def diferenca_angular(angle1, angle2):
    diff = (angle1 - angle2 + 180) % 360 - 180
    return abs(diff)

def escolher_direcao(densidade, distancias, largura_robo=0.30, objetivo_direcao=0, resolucao_angular=10):
    num_setores = len(densidade)

    # Marca setores livres
    livres = [dist > (largura_robo / 2) and dist > 0.1 for dist in distancias]

    # --- ALTERAÇÃO AQUI ---
    # Reorganiza os setores para começar no índice correspondente a 270°
    inicio_scan = int(270 // resolucao_angular)
    livres = livres[inicio_scan:] + livres[:inicio_scan]

    # Encontrar blocos contínuos
    blocos = []
    inicio = None
    for i in range(num_setores):
        if livres[i]:
            if inicio is None:
                inicio = i
        else:
            if inicio is not None:
                blocos.append((inicio, i - 1))
                inicio = None
    if inicio is not None:
        blocos.append((inicio, num_setores - 1))

    if not blocos:
        return None

    # Bloco mais largo
    melhor_bloco = max(blocos, key=lambda b: (b[1] - b[0] + 1) % num_setores)
    inicio, fim = melhor_bloco

    print(f'Blocos: {blocos}')
    print(f'Melhor bloco: {melhor_bloco}')

    # Centro do bloco
    if fim >= inicio:
        centro = (inicio + fim) // 2
        if math.fabs(fim + inicio) % 2 == 1:
            centro += 1
    else:
        tamanho_bloco = ((fim + num_setores) - inicio + 1)
        centro = (inicio + tamanho_bloco // 2) % num_setores

    # --- DESFAZ O REORDENAMENTO ---
    centro_original = (centro + inicio_scan) % num_setores

    if math.fabs(fim - inicio) % 2 == 0:
        return centro_original * resolucao_angular + (resolucao_angular / 2)

    return centro_original * resolucao_angular


class ControladorRobo(Node):
    def __init__(self):
        super().__init__('control_node')
        self.get_logger().info('Nó principal iniciado')
        # Lógica principal aqui
        self.lidar_cli = self.create_client(LidarResponse, 'get_lidar_points')
        self.movement_cli = self.create_client(TimedMoveResponse, 'send_move_command')

    def call_lidar_service(self):
        req = LidarResponse.Request()

        future = self.lidar_cli.call_async(req)
        # Espera até o resultado chegar (síncrono)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result().points
        else:
            raise RuntimeError('Falha na chamada ao serviço')
        
    def call_timed_move_service(self, distance, angle, orientation_sign):
        req = TimedMoveResponse.Request()
        req.distance = distance
        req.angle = angle
        req.orientation_sign = orientation_sign

        future = self.movement_cli.call_async(req)
        # Espera até o resultado chegar (síncrono)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return
        else:
            raise RuntimeError('Falha na chamada ao serviço')

    def executar_logica(self):
        lidar_points = self.call_lidar_service()
        angles, distances = convert_from_points_to_lists(lidar_points)
        direction, orientation, distance = vfh(angles, distances)
        self.call_timed_move_service(distance, direction, orientation)


def main(args=None):
    rclpy.init(args=args)
    controlador = ControladorRobo()

    try:
        while rclpy.ok():
            controlador.executar_logica()
            rclpy.spin_once(controlador, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    except EndOfPath:
        print("Cheguei no meu destino")
        pass
    finally:
        controlador.destroy_node()
        rclpy.shutdown()
