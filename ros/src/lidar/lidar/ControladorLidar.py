from project_interfaces.srv import LidarResponse
from project_interfaces.msg import LidarPoint
import rclpy
from rclpy.node import Node
from .logics.lidar_integration import executar_python_e_salvar_saida, proccess_data

# import subprocess

# def executar_python_e_salvar_saida(arquivo_python: str, arquivo_saida: str):
#     """
#     Executa um script Python e salva todas as saídas (stdout e stderr) em um arquivo txt.

#     :param arquivo_python: Caminho do arquivo Python a ser executado.
#     :param arquivo_saida: Caminho do arquivo txt onde será salvo o output.
#     """
#     try:
#         # Executa o arquivo Python e captura a saída padrão e erros
#         resultado = subprocess.run(
#             ['python3', arquivo_python],
#             stdout=subprocess.PIPE,
#             stderr=subprocess.PIPE,
#             text=True  # para obter saída em string (não bytes)
#         )
        
#         # Junta stdout e stderr
#         saida_completa = f"--- STDOUT ---\n{resultado.stdout}\n--- STDERR ---\n{resultado.stderr}"
        
#         # Escreve no arquivo txt
#         with open(arquivo_saida, 'w', encoding='utf-8') as f:
#             f.write(saida_completa)
            
#         print(f"Saída salva em {arquivo_saida}")
    
#     except Exception as e:
#         print(f"Erro ao executar o arquivo: {e}")

def convertToLidarPoint(angle, range):
    point = LidarPoint()
    point.angle = angle
    point.distance = range

    return point

class MinimalService(Node):

    def __init__(self):
        super().__init__('lidar_service')
        self.srv = self.create_service(LidarResponse, 
                                       'get_lidar_points', 
                                       self.get_lidar_points_callback)

    def get_lidar_points_callback(self, request, response):
        executar_python_e_salvar_saida("./src/lidar/lidar/logics/lidar.py", "/home/robot/Documents/tests/result.txt")
        angles, ranges = proccess_data()
        # scan_environment()
        # zipped_data = list(zip(angles, distances))
        lidarPoints = [convertToLidarPoint(angles[i], ranges[i]) for i in range(len(angles))]
        response.points = lidarPoints
        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
