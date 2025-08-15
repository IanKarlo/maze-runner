import subprocess
import csv
import numpy as np
import matplotlib.pyplot as plt
import math

def executar_python_e_salvar_saida(arquivo_python: str, arquivo_saida: str):
    """
    Executa um script Python e salva todas as saídas (stdout e stderr) em um arquivo txt.

    :param arquivo_python: Caminho do arquivo Python a ser executado.
    :param arquivo_saida: Caminho do arquivo txt onde será salvo o output.
    """
    try:
        # Executa o arquivo Python e captura a saída padrão e erros
        resultado = subprocess.run(
            ['python3', arquivo_python],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True  # para obter saída em string (não bytes)
        )
        
        # Junta stdout e stderr
        saida_completa = f"--- STDOUT ---\n{resultado.stdout}\n--- STDERR ---\n{resultado.stderr}"
        
        print("Execução do lidar concluída com sucesso")
    
    except Exception as e:
        print(f"Erro ao executar o arquivo: {e}")

def proccess_data():
    # Caminho do arquivo CSV
    arquivo_csv = "dados_lidar.csv"

    angles = []
    ranges = []

    with open(arquivo_csv, newline='') as csvfile:
        leitor = csv.DictReader(csvfile)
        for linha in leitor:
            ang = float(linha["Angle"])
            ran = float(linha["Range"])
            angles.append(ang)
            # Aplica o filtro para ranges >= 32
            if ran < 32:
                ranges.append(ran)
            else:
                ranges.append(0)

    # Converte os ângulos para radianos e aplica o ajuste
    angles_rad = [math.pi + ang - math.pi/3 for ang in angles]

    # Filtro por ângulo após atualização
    angulos_filtrados = []
    distancias_filtradas = []
    for ang, dist in zip(angles_rad, ranges):
        ang_deg = math.degrees(ang) % 360
        if not (90 <= ang_deg <= 270):
            angulos_filtrados.append(ang)
            distancias_filtradas.append(dist)

    # Mantém tratamento de distâncias
    distancias_filtradas = [0.3 if x <= 0 else x if x < 2 else 10.0 for x in distancias_filtradas]

    return angulos_filtrados, distancias_filtradas