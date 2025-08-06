import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import math

# Caminho do arquivo CSV
arquivo_csv = "dados_lidar.csv"

# Lê os dados do CSV
dados = pd.read_csv(arquivo_csv)

# Extrai ângulo e range
angles = dados["Angle"].values         # Em graus
ranges = dados["Range"].values         # Em metros (ou unidade do LIDAR)

# Converte os ângulos para radianos
angles_rad = list(map(lambda x: math.pi + x - math.pi/3, angles))
ranges = list(map(lambda x: x if x < 32 else 0, ranges))

# Cria o gráfico polar
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, polar=True)

# Plota os pontos
ax.scatter(angles_rad, ranges, s=5, c='blue', alpha=0.7, label="Leitura LIDAR")

# Personalizações
ax.set_theta_zero_location('N')   # 0° para cima
ax.set_theta_direction(-1)        # Sentido horário
ax.set_rlabel_position(135)       # Posição dos rótulos do raio
ax.set_title("Gráfico de Radar - Dados LIDAR (CSV)", va='bottom')

plt.legend()
plt.grid(True)
plt.show()
