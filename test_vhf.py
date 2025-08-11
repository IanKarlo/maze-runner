# import csv
# import math
# import numpy as np
# import matplotlib.pyplot as plt

# largura_robo = 0.65              # Largura do robô em metros
# resolucao_angular = 20            # Tamanho de cada setor (graus)
# num_setores = 360 // resolucao_angular  # Total de setores

# # === 1. Lê o CSV manualmente ===
# def ler_csv(filepath):
#     angulos = []
#     distancias = []
#     with open(filepath, newline='') as csvfile:
#         reader = csv.reader(csvfile)
#         next(reader)  # pula o cabeçalho
#         for row in reader:
#             if len(row) < 2:
#                 continue
#             try:
#                 ang_rad = float(row[0])
#                 dist = float(row[1])
#                 if dist > 0:  # ignora leitura zero
#                     angulos.append(ang_rad)  # radiano
#                     distancias.append(dist)
#             except ValueError:
#                 continue
    
#     res = len(list(filter(lambda x: x < 0.1, distancias)))
#     print(f'res: {res}')

#     angulos = [math.pi + ang - math.pi/3 for ang in angulos]
#     distancias = [0.3 if x <= 0 else x if x < 2 else 10 for x in distancias]
#     return angulos, distancias

# # === 2. Cria vetor de distâncias por setor ===
# def criar_vetor_distancias(angulos_rad, medidas, resolucao_angular):
#     num_setores = 360 // resolucao_angular
#     distancias = np.ones(num_setores) * 10.0  # valor alto = sem obstáculo
#     for ang_rad, dist in zip(angulos_rad, medidas):
#         ang_deg = math.degrees(ang_rad) % 360
#         indice = int(ang_deg // resolucao_angular)
#         distancias[indice] = min(distancias[indice], dist)
#     return distancias

# # === 3. Histograma de densidade inversa ===
# def construir_histograma(distancias, limiar=1.5):
#     densidade = np.where(distancias < limiar, 1 / distancias, 0)
#     return densidade

# # === 4. Escolha da direção ideal considerando a largura do robô ===
# def diferenca_angular(angle1, angle2):
#     """Calcula a diferença mínima entre dois ângulos em graus no intervalo [-180, 180]."""
#     diff = (angle1 - angle2 + 180) % 360 - 180
#     return abs(diff)

# def escolher_direcao(densidade, distancias, largura_robo=0.30, objetivo_direcao=0, resolucao_angular=10):
#     num_setores = len(densidade)
#     candidatos = []

#     for i in range(num_setores):
#         distancia = distancias[i]
#         if distancia <= 0.1:
#             continue
#         try:
#             angulo_necessario_rad = 2 * math.atan((largura_robo / 2) / distancia)
#             abertura = int(math.degrees(angulo_necessario_rad) // resolucao_angular)
#         except:
#             continue

#         abertura = max(abertura, 1)
#         inicio = max(i - abertura // 2, 0)
#         fim = min(i + abertura // 2 + 1, num_setores - 1)
#         janela = densidade[inicio:fim]
#         if np.sum(janela) < 1.0:
#             candidatos.append(i)

#     if not candidatos:
#         return None

#     melhor_indice = min(
#         candidatos,
#         key=lambda x: diferenca_angular(x * resolucao_angular, objetivo_direcao)
#     )
#     return melhor_indice * resolucao_angular


# # === 5. Plotar gráficos lado a lado ===
# def plotar_comparativo(angulos_rad, medidas, distancias_por_setor, direcao=None, resolucao_angular=5):
#     angles_hist = np.radians(np.arange(0, 360, resolucao_angular))

#     fig, axs = plt.subplots(1, 2, subplot_kw={'projection': 'polar'}, figsize=(12, 6))

#     # Esquerda: Dispersão dos pontos
#     axs[0].scatter(angulos_rad, medidas, s=10, c='green', alpha=0.7)
#     axs[0].set_title("Dispersão das Leituras (LIDAR)")
#     axs[0].set_theta_zero_location("N")  # 0° apontando para cima
#     axs[0].set_theta_direction(-1)        # sentido anti-horário
#     axs[0].set_rlim(0, 2.0) 

#     # Direita: Histograma polar com setores visíveis
#     larguras = np.radians(resolucao_angular)
#     angles = np.radians(np.arange(0, 360, resolucao_angular))
#     axs[1].bar(angles, distancias_por_setor, width=larguras, bottom=0.0, alpha=0.6, edgecolor='black', align='edge', color='skyblue')

#     # Marca a direção sugerida (se houver)
#     if direcao is not None:
#         dir_rad = math.radians(direcao)
#         setor_index = int(direcao // resolucao_angular)
#         distancia_sugerida = min(distancias_por_setor[setor_index], 1.7)
#         print(f'distancia_sugerida={distancias_por_setor[setor_index]}')
#         axs[1].plot(dir_rad, distancia_sugerida, 'ro', label="Direção sugerida")

#     axs[1].set_title("Histograma Polar com Setores")
#     axs[1].set_theta_zero_location("N")
#     axs[1].set_theta_direction(-1)
#     axs[1].set_rlim(0, 2.0) 
#     axs[1].legend()


#     plt.tight_layout()
#     plt.show()

# # === Execução principal ===
# arquivo_csv = "dados_lidar.csv"
# angulos_rad, medidas = ler_csv(arquivo_csv)
# distancias_por_setor = criar_vetor_distancias(angulos_rad, medidas, resolucao_angular)
# densidade = construir_histograma(distancias_por_setor)
# direcao = escolher_direcao(densidade, distancias_por_setor, largura_robo, objetivo_direcao=0, resolucao_angular=resolucao_angular)

# print(f"Direção sugerida: {direcao}°")
# plotar_comparativo(angulos_rad, medidas, distancias_por_setor, direcao, resolucao_angular)


# import csv
# import math
# import numpy as np
# import matplotlib.pyplot as plt

# largura_robo = 0.15              # Largura do robô em metros
# resolucao_angular = 5            # Tamanho de cada setor (graus)
# num_setores = 360 // resolucao_angular  # Total de setores
# limiar_densidade = 1/0.2           # Limiar para considerar setor livre

# # === 1. Lê o CSV ===
# def ler_csv(filepath):
#     angulos = []
#     distancias = []
#     with open(filepath, newline='') as csvfile:
#         reader = csv.reader(csvfile)
#         next(reader)  # pula o cabeçalho
#         for row in reader:
#             if len(row) < 2:
#                 continue
#             try:
#                 ang_rad = float(row[0])
#                 dist = float(row[1])
#                 if dist > 0:  # ignora leitura zero
#                     angulos.append(ang_rad)  # radiano
#                     distancias.append(dist)
#             except ValueError:
#                 continue

#     # Compensação do posicionamento do LIDAR no robô
#     angulos = [math.pi + ang - math.pi/3 for ang in angulos]

#     # Normalização (pode ser desativada se não precisar)
#     distancias = [x if x < 2 else 10 for x in distancias]
#     return angulos, distancias

# # === 2. Cria vetor de distâncias por setor ===
# def criar_vetor_distancias(angulos_rad, medidas, resolucao_angular):
#     num_setores = 360 // resolucao_angular
#     distancias = np.ones(num_setores) * 10.0  # valor alto = sem obstáculo
#     for ang_rad, dist in zip(angulos_rad, medidas):
#         ang_deg = math.degrees(ang_rad) % 360
#         indice = int(ang_deg // resolucao_angular)
#         distancias[indice] = min(distancias[indice], dist)
#     return distancias

# # === 3. Histograma de densidade com suavização ===
# def construir_histograma(distancias, limiar=1.5):
#     densidade = np.where(distancias < limiar, 1 / (distancias + 1e-6), 0)

#     # Filtro de suavização (média de 3 setores circulares)
#     dens_suave = np.copy(densidade)
#     for i in range(len(densidade)):
#         vizinhos = [
#             densidade[(i - 1) % len(densidade)],
#             densidade[i],
#             densidade[(i + 1) % len(densidade)]
#         ]
#         dens_suave[i] = np.mean(vizinhos)

#     return dens_suave

# # === 4. Função utilitária ===
# def diferenca_angular(angle1, angle2):
#     diff = (angle1 - angle2 + 180) % 360 - 180
#     return abs(diff)

# # === 5. Escolha da direção ideal com circularidade ===
# def escolher_direcao(densidade, distancias, largura_robo=0.30, objetivo_direcao=0, resolucao_angular=10, limiar=0.5):
#     num_setores = len(densidade)
#     candidatos = []

#     for i in range(num_setores):
#         distancia = distancias[i]
#         if distancia <= 0.1:
#             continue
#         try:
#             angulo_necessario_rad = 2 * math.atan((largura_robo / 2) / distancia)
#             abertura = int(math.degrees(angulo_necessario_rad) // resolucao_angular)
#         except:
#             continue

#         abertura = max(abertura, 1)

#         # Circularidade: considerar setores vizinhos corretamente
#         indices_janela = [(i + k) % num_setores for k in range(-abertura // 2, abertura // 2 + 1)]
#         janela = [densidade[j] for j in indices_janela]

#         # Critério: todos os setores da janela devem ter densidade abaixo do limiar
#         if all(v < limiar for v in janela):
#             candidatos.append(i)

#     if not candidatos:
#         return None

#     melhor_indice = min(
#         candidatos,
#         key=lambda x: diferenca_angular(x * resolucao_angular, objetivo_direcao)
#     )
#     return melhor_indice * resolucao_angular

# # === 6. Plotagem com densidade ===
# def plotar_comparativo(angulos_rad, medidas, densidade, direcao=None, resolucao_angular=5, limiar=0.5):
#     fig, axs = plt.subplots(1, 2, subplot_kw={'projection': 'polar'}, figsize=(12, 6))

#     # --- Dispersão das leituras ---
#     axs[0].scatter(angulos_rad, medidas, s=10, c='green', alpha=0.7)
#     axs[0].set_title("Dispersão das Leituras (LIDAR)")
#     axs[0].set_theta_zero_location("N")
#     axs[0].set_theta_direction(-1)
#     axs[0].set_rlim(0, 2.0) 

#     # --- Histograma de densidade ---
#     larguras = np.radians(resolucao_angular)
#     angles = np.radians(np.arange(0, 360, resolucao_angular))
#     axs[1].bar(angles, densidade, width=larguras, bottom=0.0, alpha=0.6,
#                edgecolor='black', align='edge', color='orange', label='Densidade')

#     axs[1].plot([0, 2 * np.pi], [limiar, limiar], 'r--', label=f"Limiar ({limiar})")

#     if direcao is not None:
#         dir_rad = math.radians(direcao)
#         setor_index = int(direcao // resolucao_angular)
#         densidade_sugerida = densidade[setor_index]
#         axs[1].plot(dir_rad, densidade_sugerida, 'bo', markersize=8, label="Direção sugerida")

#     axs[1].set_title("Histograma Polar de Densidade")
#     axs[1].set_theta_zero_location("N")
#     axs[1].set_theta_direction(-1)
#     axs[1].set_rlim(0, max(densidade) * 1.2 if max(densidade) > 0 else 1)
#     axs[1].legend()

#     plt.tight_layout()
#     plt.show()

# # === 7. Execução principal ===
# arquivo_csv = "dados_lidar.csv"
# angulos_rad, medidas = ler_csv(arquivo_csv)
# distancias_por_setor = criar_vetor_distancias(angulos_rad, medidas, resolucao_angular)
# densidade = construir_histograma(distancias_por_setor)
# direcao = escolher_direcao(densidade, distancias_por_setor, largura_robo, objetivo_direcao=0,
#                             resolucao_angular=resolucao_angular, limiar=limiar_densidade)

# print(f"Direção sugerida: {direcao}°")
# plotar_comparativo(angulos_rad, medidas, densidade, direcao, resolucao_angular, limiar_densidade)


# import csv
# import math
# import numpy as np
# import matplotlib.pyplot as plt

# # ====================
# # Configurações do robô e LIDAR
# # ====================
# largura_robo = 0.65              # metros
# margem_lateral_segura = 0.20     # metros
# distancia_segura_frontal = 0.20  # metros
# resolucao_angular = 5            # graus por setor
# num_setores = 360 // resolucao_angular


# # ====================
# # 1. Leitura do CSV
# # ====================
# def ler_csv(filepath):
#     angulos = []
#     distancias = []
#     with open(filepath, newline='') as csvfile:
#         reader = csv.reader(csvfile)
#         next(reader)  # pula cabeçalho
#         for row in reader:
#             if len(row) < 2:
#                 continue
#             try:
#                 ang_rad = float(row[0])
#                 dist = float(row[1])
#                 if dist > 0:
#                     angulos.append(ang_rad)  # radianos
#                     distancias.append(dist)
#             except ValueError:
#                 continue

#     # Ajuste do ângulo para alinhar com o centro do robô
#     angulos = [math.pi + ang - math.pi/3 for ang in angulos]

#     # Limita e normaliza distâncias (teste)
#     distancias = [0.3 if x <= 0 else x if x < 2 else 10 for x in distancias]
#     return angulos, distancias


# # ====================
# # 2. Setorização
# # ====================
# def criar_vetor_distancias(angulos_rad, medidas, resolucao_angular):
#     num_setores = 360 // resolucao_angular
#     distancias = np.ones(num_setores) * 10.0  # valor alto = livre
#     for ang_rad, dist in zip(angulos_rad, medidas):
#         ang_deg = math.degrees(ang_rad) % 360
#         indice = int(ang_deg // resolucao_angular)
#         distancias[indice] = min(distancias[indice], dist)
#     return distancias


# # ====================
# # 3. Histograma de densidade inversa
# # ====================
# def construir_histograma(distancias, distancia_segura):
#     limiar = 1.0 / distancia_segura
#     densidade = np.where(distancias < distancia_segura, 1 / distancias, 0)
#     return densidade, limiar


# # ====================
# # 4. Escolha de direção (movimento quadrado)
# # ====================
# def diferenca_angular(angle1, angle2):
#     diff = (angle1 - angle2 + 180) % 360 - 180
#     return abs(diff)

# def abertura_em_setores_por_distancia(distancia, largura_robo, margem, resolucao_angular):
#     if distancia <= 0:
#         return np.inf
#     angulo_necessario_rad = 2 * math.atan((largura_robo / 2 + margem) / distancia)
#     return max(int(math.degrees(angulo_necessario_rad) // resolucao_angular), 1)

# def escolher_direcao(densidade, distancias, largura_robo, margem_lateral, distancia_segura_frontal, objetivo_direcao, resolucao_angular):
#     num_setores = len(densidade)
#     candidatos = []

#     for i in range(num_setores):
#         distancia_frente = distancias[i]
#         if distancia_frente < distancia_segura_frontal:
#             continue

#         abertura = abertura_em_setores_por_distancia(distancia_frente, largura_robo, margem_lateral, resolucao_angular)

#         inicio = max(i - abertura // 2, 0)
#         fim = min(i + abertura // 2 + 1, num_setores)
#         janela = densidade[inicio:fim]

#         # Livre se não tem obstáculo na janela lateral
#         if np.sum(janela) == 0:
#             candidatos.append(i)

#     if not candidatos:
#         return None

#     melhor_indice = min(candidatos, key=lambda x: diferenca_angular(x * resolucao_angular, objetivo_direcao))
#     return melhor_indice * resolucao_angular


# # ====================
# # 5. Plotagem
# # ====================
# def plotar_comparativo(angulos_rad, medidas, distancias_por_setor, direcao=None, resolucao_angular=5):
#     fig, axs = plt.subplots(1, 2, subplot_kw={'projection': 'polar'}, figsize=(12, 6))

#     # Dispersão do LIDAR
#     axs[0].scatter(angulos_rad, medidas, s=10, c='green', alpha=0.7)
#     axs[0].set_title("Leituras LIDAR")
#     axs[0].set_theta_zero_location("N")
#     axs[0].set_theta_direction(-1)
#     axs[0].set_rlim(0, 2.0)

#     # Histograma polar
#     larguras = np.radians(resolucao_angular)
#     angles = np.radians(np.arange(0, 360, resolucao_angular))
#     axs[1].bar(angles, distancias_por_setor, width=larguras, bottom=0.0,
#                alpha=0.6, edgecolor='black', align='edge', color='skyblue')

#     if direcao is not None:
#         dir_rad = math.radians(direcao)
#         setor_index = int(direcao // resolucao_angular)
#         distancia_sugerida = min(distancias_por_setor[setor_index], 1.7)
#         axs[1].plot(dir_rad, distancia_sugerida, 'ro', label="Direção sugerida")

#     axs[1].set_title("Histograma Polar")
#     axs[1].set_theta_zero_location("N")
#     axs[1].set_theta_direction(-1)
#     axs[1].set_rlim(0, 2.0)
#     axs[1].legend()

#     plt.tight_layout()
#     plt.show()


# # ====================
# # Execução principal
# # ====================
# if __name__ == "__main__":
#     arquivo_csv = "dados_lidar.csv"
#     angulos_rad, medidas = ler_csv(arquivo_csv)

#     distancias_por_setor = criar_vetor_distancias(angulos_rad, medidas, resolucao_angular)
#     densidade, limiar = construir_histograma(distancias_por_setor, distancia_segura_frontal)

#     direcao = escolher_direcao(
#         densidade,
#         distancias_por_setor,
#         largura_robo,
#         margem_lateral_segura,
#         distancia_segura_frontal,
#         objetivo_direcao=0,
#         resolucao_angular=resolucao_angular
#     )

#     print(f"Direção sugerida: {direcao}°")
#     plotar_comparativo(angulos_rad, medidas, distancias_por_setor, direcao, resolucao_angular)

# import csv
# import math
# import numpy as np
# import matplotlib.pyplot as plt

# # ====================
# # Configurações do robô e LIDAR
# # ====================
# largura_robo = 0.65              # metros
# margem_lateral_segura = 0.20     # metros
# distancia_segura_frontal = 0.25  # metros
# resolucao_angular = 5            # graus por setor
# num_setores = 360 // resolucao_angular


# # ====================
# # 1. Leitura do CSV
# # ====================
# def ler_csv(filepath):
#     angulos = []
#     distancias = []
#     with open(filepath, newline='') as csvfile:
#         reader = csv.reader(csvfile)
#         next(reader)  # pula cabeçalho
#         for row in reader:
#             if len(row) < 2:
#                 continue
#             try:
#                 ang_rad = float(row[0])
#                 dist = float(row[1])
#                 if dist > 0:
#                     angulos.append(ang_rad)  # radianos
#                     distancias.append(dist)
#             except ValueError:
#                 continue

#     # Ajuste do ângulo para alinhar com o centro do robô
#     angulos = [math.pi + ang - math.pi/3 for ang in angulos]

#     # Limita e normaliza distâncias
#     distancias = [0.3 if x <= 0 else x if x < 2 else 10 for x in distancias]
#     return angulos, distancias


# # ====================
# # 2. Setorização
# # ====================
# def criar_vetor_distancias(angulos_rad, medidas, resolucao_angular):
#     num_setores = 360 // resolucao_angular
#     distancias = np.ones(num_setores) * 10.0  # valor alto = livre
#     for ang_rad, dist in zip(angulos_rad, medidas):
#         ang_deg = math.degrees(ang_rad) % 360
#         indice = int(ang_deg // resolucao_angular)
#         distancias[indice] = min(distancias[indice], dist)
#     return distancias


# # ====================
# # 3. Histograma de densidade inversa
# # ====================
# def construir_histograma(distancias, distancia_segura):
#     limiar = 1.0 / distancia_segura
#     densidade = np.where(distancias < distancia_segura, 1 / distancias, 0)
#     return densidade, limiar


# # ====================
# # 4. Escolha de direção (movimento quadrado com verificação lateral contínua)
# # ====================
# def diferenca_angular(angle1, angle2):
#     diff = (angle1 - angle2 + 180) % 360 - 180
#     return abs(diff)

# def abertura_em_setores_por_distancia(distancia, largura_robo, margem, resolucao_angular):
#     if distancia <= 0:
#         return np.inf
#     angulo_necessario_rad = 2 * math.atan((largura_robo / 2 + margem) / distancia)
#     return max(int(math.degrees(angulo_necessario_rad) // resolucao_angular), 1)

# def escolher_direcao(densidade, distancias, largura_robo, margem_lateral,
#                      distancia_segura_frontal, objetivo_direcao, resolucao_angular):
#     num_setores = len(densidade)
#     candidatos = []

#     for i in range(num_setores):
#         distancia_frente = distancias[i]
#         if distancia_frente < distancia_segura_frontal:
#             continue

#         # Quantos setores para cada lado precisamos livres para passar
#         abertura = abertura_em_setores_por_distancia(distancia_frente, largura_robo, margem_lateral, resolucao_angular)

#         # Setores à frente e laterais que o robô ocuparia
#         inicio = max(i - abertura // 2, 0)
#         fim = min(i + abertura // 2 + 1, num_setores)
#         janela = densidade[inicio:fim]

#         # Verificação lateral contínua
#         seguro = True
#         passos = int(distancia_segura_frontal / 0.05)  # passos de 5 cm
#         for passo in range(1, passos + 1):
#             dist_check = passo * 0.05
#             abertura_passos = abertura_em_setores_por_distancia(dist_check, largura_robo, margem_lateral, resolucao_angular)
#             inicio_check = max(i - abertura_passos // 2, 0)
#             fim_check = min(i + abertura_passos // 2 + 1, num_setores)

#             if np.any(distancias[inicio_check:fim_check] < dist_check):
#                 seguro = False
#                 break

#         if seguro and np.sum(janela) == 0:
#             candidatos.append(i)

#     if not candidatos:
#         return None

#     # Escolhe candidato mais próximo do objetivo
#     melhor_indice = min(candidatos, key=lambda x: diferenca_angular(x * resolucao_angular, objetivo_direcao))
#     return melhor_indice * resolucao_angular


# # ====================
# # 5. Plotagem
# # ====================
# def plotar_comparativo(angulos_rad, medidas, distancias_por_setor, direcao=None, resolucao_angular=5):
#     fig, axs = plt.subplots(1, 2, subplot_kw={'projection': 'polar'}, figsize=(12, 6))

#     # Dispersão do LIDAR
#     axs[0].scatter(angulos_rad, medidas, s=10, c='green', alpha=0.7)
#     axs[0].set_title("Leituras LIDAR")
#     axs[0].set_theta_zero_location("N")
#     axs[0].set_theta_direction(-1)
#     axs[0].set_rlim(0, 2.0)

#     # Histograma polar
#     larguras = np.radians(resolucao_angular)
#     angles = np.radians(np.arange(0, 360, resolucao_angular))
#     axs[1].bar(angles, distancias_por_setor, width=larguras, bottom=0.0,
#                alpha=0.6, edgecolor='black', align='edge', color='skyblue')

#     if direcao is not None:
#         dir_rad = math.radians(direcao)
#         setor_index = int(direcao // resolucao_angular)
#         distancia_sugerida = min(distancias_por_setor[setor_index], 1.7)
#         axs[1].plot(dir_rad, distancia_sugerida, 'ro', label="Direção sugerida")

#     axs[1].set_title("Histograma Polar")
#     axs[1].set_theta_zero_location("N")
#     axs[1].set_theta_direction(-1)
#     axs[1].set_rlim(0, 2.0)
#     axs[1].legend()

#     plt.tight_layout()
#     plt.show()


# # ====================
# # Execução principal
# # ====================
# if __name__ == "__main__":
#     arquivo_csv = "dados_lidar.csv"
#     angulos_rad, medidas = ler_csv(arquivo_csv)

#     distancias_por_setor = criar_vetor_distancias(angulos_rad, medidas, resolucao_angular)
#     densidade, limiar = construir_histograma(distancias_por_setor, distancia_segura_frontal)

#     direcao = escolher_direcao(
#         densidade,
#         distancias_por_setor,
#         largura_robo,
#         margem_lateral_segura,
#         distancia_segura_frontal,
#         objetivo_direcao=0,
#         resolucao_angular=resolucao_angular
#     )

#     print(f"Direção sugerida: {direcao}°")
#     plotar_comparativo(angulos_rad, medidas, distancias_por_setor, direcao, resolucao_angular)

# import csv
# import math
# import numpy as np
# import matplotlib.pyplot as plt

# # ====================
# # Configurações do robô e LIDAR
# # ====================
# largura_robo = 0.35              # metros
# margem_lateral_segura = 0.20     # metros
# distancia_segura_frontal = 0.25  # metros
# resolucao_angular = 5            # graus por setor
# num_setores = 360 // resolucao_angular

# # Parâmetros novos
# distancia_movimento = 1.0   # metros de lookahead (quanto o robô pretende andar após girar)
# passo_sim = 0.05            # passo da simulação no lookahead (m)


# # ====================
# # 1. Leitura do CSV
# # ====================
# def ler_csv(filepath):
#     angulos = []
#     distancias = []
#     with open(filepath, newline='') as csvfile:
#         reader = csv.reader(csvfile)
#         next(reader)  # pula cabeçalho
#         for row in reader:
#             if len(row) < 2:
#                 continue
#             try:
#                 ang_rad = float(row[0])
#                 dist = float(row[1])
#                 if dist > 0:
#                     angulos.append(ang_rad)  # radianos
#                     distancias.append(dist)
#             except ValueError:
#                 continue

#     # Ajuste do ângulo para alinhar com o centro do robô (preservado conforme seu código)
#     angulos = [math.pi + ang - math.pi/3 for ang in angulos]

#     # Limita e normaliza distâncias (mantive seu teste)
#     distancias = [0.3 if x <= 0 else x if x < 2 else 10 for x in distancias]
#     return angulos, distancias


# # ====================
# # 2. Setorização
# # ====================
# def criar_vetor_distancias(angulos_rad, medidas, resolucao_angular):
#     num_setores_local = 360 // resolucao_angular
#     distancias = np.ones(num_setores_local) * 10.0  # valor alto = livre
#     for ang_rad, dist in zip(angulos_rad, medidas):
#         ang_deg = math.degrees(ang_rad) % 360
#         indice = int(ang_deg // resolucao_angular)
#         distancias[indice] = min(distancias[indice], dist)
#     return distancias


# # ====================
# # 3. Histograma de densidade inversa
# # ====================
# def construir_histograma(distancias, distancia_segura):
#     limiar = 1.0 / distancia_segura
#     densidade = np.where(distancias < distancia_segura, 1 / (distancias + 1e-6), 0)
#     return densidade, limiar


# # ====================
# # 4. Escolha de direção (movimento quadrado com lookahead lateral contínuo)
# # ====================
# def diferenca_angular(angle1, angle2):
#     diff = (angle1 - angle2 + 180) % 360 - 180
#     return abs(diff)

# def abertura_em_setores_por_distancia(distancia, largura_robo, margem, resolucao_angular):
#     """
#     Número de setores necessários para que o robô (com margem) passe a certa distância.
#     Retorna pelo menos 1 e no máximo num_setores.
#     """
#     if distancia <= 0:
#         return num_setores  # impossível: usa todo o círculo como "necessário"
#     r_efetivo = largura_robo / 2.0 + margem
#     # ângulo necessário (rad)
#     ang = 2.0 * math.atan2(r_efetivo, distancia)
#     deg = math.degrees(ang)
#     setores = int(math.ceil(deg / resolucao_angular))
#     setores = max(1, setores)
#     setores = min(setores, num_setores)
#     return setores

# def _indices_janela(center_idx, abertura, num_setores):
#     """
#     Retorna lista de índices modulados que cobrem a janela centralizada em center_idx com 'abertura' setores.
#     """
#     half = abertura // 2
#     # se abertura for par, isso ainda dá uma janela simétrica aproximada
#     return [ (center_idx + offset) % num_setores for offset in range(-half, half + 1) ]

# def escolher_direcao(densidade, distancias, largura_robo, margem_lateral,
#                      distancia_segura_frontal, objetivo_direcao, resolucao_angular,
#                      distancia_movimento=1.0, passo=0.05):
#     """
#     Retorna: (direcao_degrees ou None, lista_de_candidatos)
#     cada candidato é (index_setor, folga_minima_em_metros)
#     """
#     num_setores_local = len(densidade)
#     candidatos = []

#     for i in range(num_setores_local):
#         distancia_frente = distancias[i]
#         # exige segurança frontal imediata
#         if distancia_frente < distancia_segura_frontal:
#             continue

#         # abertura necessária na distância frontal
#         abertura = abertura_em_setores_por_distancia(distancia_frente, largura_robo, margem_lateral, resolucao_angular)
#         indices_janela = _indices_janela(i, abertura, num_setores_local)

#         # janela frontal deve estar "livre" no limiar frontal (densidade==0)
#         if np.any(densidade[indices_janela] > 0):
#             continue

#         # Lookahead lateral contínuo: percorre passos até distancia_movimento
#         seguro = True
#         folga_minima = float("inf")
#         num_passos = max(1, int(math.ceil(distancia_movimento / passo)))
#         for passo_idx in range(1, num_passos + 1):
#             dist_check = passo_idx * passo
#             if dist_check > distancia_movimento:
#                 dist_check = distancia_movimento

#             abertura_check = abertura_em_setores_por_distancia(dist_check, largura_robo, margem_lateral, resolucao_angular)
#             indices_check = _indices_janela(i, abertura_check, num_setores_local)

#             # pega as leituras correspondentes (circular)
#             leituras = distancias[indices_check]

#             # se qualquer leitura está mais próxima que o ponto que vamos checar, risco de colisão
#             if np.any(leituras < dist_check):
#                 seguro = False
#                 break

#             # folga nesse passo = mínimo das leituras nessa janela menos dist_check
#             folga_pass = np.min(leituras) - dist_check
#             folga_minima = min(folga_minima, folga_pass)

#         if seguro:
#             # folga_minima pode ser grande (ambiente aberto); para comparação, mantemos o valor
#             candidatos.append((i, folga_minima))

#     if not candidatos:
#         return None, []

#     # ordenar candidatos: primeiro por folga (desc), empate por proximidade angular ao objetivo (asc)
#     def key_fn(t):
#         idx, folga = t
#         ang = idx * resolucao_angular
#         ang_diff = diferenca_angular(ang, objetivo_direcao)
#         # queremos folga maior (portanto -folga para ordenar desc), e ang_diff menor (asc)
#         return (-folga, ang_diff)

#     candidatos.sort(key=key_fn)
#     melhor_idx, melhor_folga = candidatos[0]
#     direcao_deg = melhor_idx * resolucao_angular
#     return direcao_deg, candidatos


# # ====================
# # 5. Plotagem (agora marca candidatos e escolhido)
# # ====================
# def plotar_comparativo(angulos_rad, medidas, distancias_por_setor, direcao=None, candidatos=None, resolucao_angular=5):
#     fig, axs = plt.subplots(1, 2, subplot_kw={'projection': 'polar'}, figsize=(12, 6))

#     # Dispersão do LIDAR
#     axs[0].scatter(angulos_rad, medidas, s=10, c='green', alpha=0.7)
#     axs[0].set_title("Leituras LIDAR")
#     axs[0].set_theta_zero_location("N")
#     axs[0].set_theta_direction(-1)
#     axs[0].set_rlim(0, 2.0)

#     # Histograma polar
#     larguras = np.radians(resolucao_angular)
#     angles = np.radians(np.arange(0, 360, resolucao_angular))

#     # pintar barras: padrão skyblue, candidatos em verde, escolhido em vermelho
#     colors = ['skyblue'] * len(distancias_por_setor)
#     if candidatos:
#         for idx, folga in candidatos:
#             colors[idx] = 'limegreen'
#     if direcao is not None:
#         setor_escolhido = int(direcao // resolucao_angular) % (360 // resolucao_angular)
#         colors[setor_escolhido] = 'red'

#     axs[1].bar(angles, distancias_por_setor, width=larguras, bottom=0.0,
#                alpha=0.8, edgecolor='black', align='edge', color=colors)

#     # legenda informativa
#     handles = []
#     handles.append(plt.Line2D([0], [0], marker='s', color='w', label='Livre', markerfacecolor='skyblue', markersize=10))
#     handles.append(plt.Line2D([0], [0], marker='s', color='w', label='Candidato', markerfacecolor='limegreen', markersize=10))
#     handles.append(plt.Line2D([0], [0], marker='s', color='w', label='Escolhido', markerfacecolor='red', markersize=10))
#     axs[1].legend(handles=handles, loc='upper right')

#     if direcao is not None:
#         dir_rad = math.radians(direcao)
#         setor_index = int(direcao // resolucao_angular)
#         distancia_sugerida = min(distancias_por_setor[setor_index], 1.7)
#         axs[1].plot(dir_rad, distancia_sugerida, 'ko')  # marca com ponto preto

#     axs[1].set_title("Histograma Polar")
#     axs[1].set_theta_zero_location("N")
#     axs[1].set_theta_direction(-1)
#     axs[1].set_rlim(0, 2.0)
#     plt.tight_layout()
#     plt.show()


# # ====================
# # Execução principal
# # ====================
# if __name__ == "__main__":
#     arquivo_csv = "dados_lidar.csv"
#     angulos_rad, medidas = ler_csv(arquivo_csv)

#     distancias_por_setor = criar_vetor_distancias(angulos_rad, medidas, resolucao_angular)
#     densidade, limiar = construir_histograma(distancias_por_setor, distancia_segura_frontal)

#     direcao, candidatos = escolher_direcao(
#         densidade,
#         distancias_por_setor,
#         largura_robo,
#         margem_lateral_segura,
#         distancia_segura_frontal,
#         objetivo_direcao=0,
#         resolucao_angular=resolucao_angular,
#         distancia_movimento=distancia_movimento,
#         passo=passo_sim
#     )

#     print(f"Direção sugerida: {direcao}°")
#     # exibe candidatos e suas folgas (útil para debug)
#     if candidatos:
#         print("Candidatos (setor, folga_metros):")
#         for idx, folga in candidatos[:20]:  # mostra até 20 primeiros
#             print(f"  setor {idx} ({idx*resolucao_angular}°) -> folga {folga:.3f} m")

#     plotar_comparativo(angulos_rad, medidas, distancias_por_setor, direcao, candidatos, resolucao_angular)

# import numpy as np
# import math
# import csv

# def build_polar_histogram(ranges, angles, num_sectors, a, b, robot_pos=(0,0)):
#     """
#     ranges, angles: arrays com medições do sensor (distância e ângulo)
#     num_sectors: número de setores angulares (por ex., 72)
#     a, b: constantes para cálculo da magnitude m = (c*^2)*(a - b*d)
#     Retorna histograma polar bruto.
#     """
#     sectors = np.zeros(num_sectors)
#     alpha = 2 * np.pi / num_sectors
#     for r, theta in zip(ranges, angles):
#         if np.isfinite(r):
#             x = r * np.cos(theta)
#             y = r * np.sin(theta)
#             d = np.hypot(x - robot_pos[0], y - robot_pos[1])
#             c = 1  # uso simplificado: certeza unitária
#             m = (c**2) * max(a - b * d, 0)
#             sector_idx = int((theta % (2*np.pi)) // alpha)
#             sectors[sector_idx] += m
#     return sectors

# def smooth_histogram(hist, window_size):
#     """
#     Aplica suavização (média móvel) com janela de +/– window_size setores.
#     """
#     kernel = np.ones(2*window_size + 1)
#     smooth = np.convolve(hist, kernel, mode='same') / kernel.sum()
#     return smooth

# def find_valleys(hist, lower_thresh):
#     """
#     Encontra sequências de setores cujo valor está abaixo do limiar.
#     Retorna lista de (start_idx, end_idx) para cada vale encontrado.
#     """
#     valleys = []
#     in_valley = False
#     start = 0
#     for idx, val in enumerate(hist):
#         if val < lower_thresh:
#             if not in_valley:
#                 in_valley = True
#                 start = idx
#         else:
#             if in_valley:
#                 valleys.append((start, idx - 1))
#                 in_valley = False
#     if in_valley:
#         valleys.append((start, len(hist) - 1))
#     return valleys

# def select_steering_direction(valleys, target_direction, num_sectors):
#     """
#     Escolhe o vale mais próximo da direção alvo e retorna o setor central.
#     target_direction em radianos.
#     """
#     alpha = 2 * np.pi / num_sectors
#     target_idx = int((target_direction % (2*np.pi)) // alpha)
#     best_valley = None
#     min_dist = num_sectors + 1
#     for (s, e) in valleys:
#         center = (s + e) // 2
#         dist = min((center - target_idx) % num_sectors,
#                    (target_idx - center) % num_sectors)
#         if dist < min_dist:
#             min_dist = dist
#             best_valley = (s, e, center)
#     if best_valley:
#         return best_valley[2] * alpha  # direção em radianos
#     else:
#         return None  # sem direção livre encontrada


# def ler_csv(filepath):
#     angulos = []
#     distancias = []
#     with open(filepath, newline='') as csvfile:
#         reader = csv.reader(csvfile)
#         next(reader)  # pula cabeçalho
#         for row in reader:
#             if len(row) < 2:
#                 continue
#             try:
#                 ang_rad = float(row[0])
#                 dist = float(row[1])
#                 if dist > 0:
#                     angulos.append(ang_rad)  # radianos
#                     distancias.append(dist)
#             except ValueError:
#                 continue

#     # Ajuste do ângulo para alinhar com o centro do robô (preservado conforme seu código)
#     angulos = [math.pi + ang - math.pi/3 for ang in angulos]

#     # Limita e normaliza distâncias (mantive seu teste)
#     distancias = [0.3 if x <= 0 else x if x < 2 else 10 for x in distancias]
#     return angulos, distancias

# # Exemplo de uso:
# if __name__ == "__main__":
#     # Simulação de dados de sensores
#     n = 360
#     arquivo_csv = "dados_lidar.csv"
#     angles, ranges =  ler_csv(arquivo_csv)

#     num_sectors = 36
#     a = 1.0
#     b = 0.1
#     raw_hist = build_polar_histogram(ranges, angles, num_sectors, a, b)
#     smooth_hist = smooth_histogram(raw_hist, window_size=2)

#     lower_thresh = 0.2
#     valleys = find_valleys(smooth_hist, lower_thresh)

#     # Supondo que o alvo esteja à frente (0 rad)
#     steer_dir = select_steering_direction(valleys, target_direction=0.0, num_sectors=num_sectors)

#     print("Vales encontrados (setor inicial, setor final):", valleys)
#     print("Direção de direção sugerida (radianos):", steer_dir)

import csv
import math
import numpy as np
import matplotlib.pyplot as plt

largura_robo = 0.65              # Largura do robô em metros
resolucao_angular = 15            # Tamanho de cada setor (graus)
num_setores = 360 // resolucao_angular  # Total de setores
DISTANCIA_MAX = 0.5               # Limite máximo de distância em metros (50 cm)

# === 1. Lê o CSV manualmente ===
def ler_csv(filepath):
    angulos = []
    distancias = []
    with open(filepath, newline='') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # pula o cabeçalho
        for row in reader:
            if len(row) < 2:
                continue
            try:
                ang_rad = float(row[0])
                dist = float(row[1])
                # Filtro de distância
                if 0 < dist <= DISTANCIA_MAX:
                    angulos.append(ang_rad)
                    distancias.append(dist)
            except ValueError:
                continue
    
    res = len(list(filter(lambda x: x < 0.1, distancias)))
    print(f'res: {res}')

    # Atualiza ângulos
    angulos = [math.pi + ang - math.pi/3 for ang in angulos]

    # Filtro por ângulo após atualização
    angulos_filtrados = []
    distancias_filtradas = []
    for ang, dist in zip(angulos, distancias):
        ang_deg = math.degrees(ang) % 360
        if not (90 <= ang_deg <= 270):
            angulos_filtrados.append(ang)
            distancias_filtradas.append(dist)

    # Mantém tratamento de distâncias
    distancias_filtradas = [0.3 if x <= 0 else x if x < 2 else 10 for x in distancias_filtradas]

    return angulos_filtrados, distancias_filtradas


# === 2. Cria vetor de distâncias por setor ===
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
    densidade = np.where(distancias < limiar, 1 / distancias, 0)
    return densidade

# === 4. Escolha da direção ideal considerando a largura do robô ===
def diferenca_angular(angle1, angle2):
    diff = (angle1 - angle2 + 180) % 360 - 180
    return abs(diff)

def escolher_direcao(densidade, distancias, largura_robo=0.30, objetivo_direcao=0, resolucao_angular=10):
    num_setores = len(densidade)

    # Marca setores livres (distância > largura do robô e não obstáculo)
    livres = [dist > (largura_robo / 2) and dist > 0.1 for dist in distancias]

    # Encontrar blocos contínuos de setores livres
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
    # Caso o bloco vá até o final e conecte com o início (setores circulares)
    if inicio is not None:
        blocos.append((inicio, num_setores - 1))

    if not blocos:
        return None  # nenhum setor livre

    # Encontrar o bloco mais largo
    melhor_bloco = max(blocos, key=lambda b: (b[1] - b[0] + 1) % num_setores)

    # Calcula o centro do bloco
    inicio, fim = melhor_bloco
    if fim >= inicio:
        centro = (inicio + fim) // 2
        if (fim - inicio) % 2 == 0:
            centro += 1
    else:
        # bloco que passa pelo zero (ex: setores 350°-10°)
        tamanho_bloco = ((fim + num_setores) - inicio + 1)
        centro = (inicio + tamanho_bloco // 2) % num_setores

    if (fim - inicio) % 2 == 0:
        return centro * resolucao_angular + (resolucao_angular / 2)

    return centro * resolucao_angular 


# === 5. Plotar gráficos lado a lado ===
def plotar_comparativo(angulos_rad, medidas, distancias_por_setor, direcao=None, resolucao_angular=5):
    angles_hist = np.radians(np.arange(0, 360, resolucao_angular))

    fig, axs = plt.subplots(1, 2, subplot_kw={'projection': 'polar'}, figsize=(12, 6))

    # Esquerda: Dispersão dos pontos
    axs[0].scatter(angulos_rad, medidas, s=10, c='green', alpha=0.7)
    axs[0].set_title("Dispersão das Leituras (LIDAR)")
    axs[0].set_theta_zero_location("N")
    axs[0].set_theta_direction(-1)
    axs[0].set_rlim(0, 2.0) 

    # Direita: Histograma polar
    larguras = np.radians(resolucao_angular)
    angles = np.radians(np.arange(0, 360, resolucao_angular))
    axs[1].bar(angles, distancias_por_setor, width=larguras, bottom=0.0, alpha=0.6, edgecolor='black', align='edge', color='skyblue')

    if direcao is not None:
        dir_rad = math.radians(direcao)
        setor_index = int(direcao // resolucao_angular)
        distancia_sugerida = min(distancias_por_setor[setor_index], 1.7)
        print(f'distancia_sugerida={distancias_por_setor[setor_index]}')
        axs[1].plot(dir_rad, distancia_sugerida, 'ro', label="Direção sugerida")

    axs[1].set_title("Histograma Polar com Setores")
    axs[1].set_theta_zero_location("N")
    axs[1].set_theta_direction(-1)
    axs[1].set_rlim(0, 2.0) 
    axs[1].legend()

    plt.tight_layout()
    plt.show()

# === Execução principal ===
arquivo_csv = "dados_lidar.csv"
angulos_rad, medidas = ler_csv(arquivo_csv)
distancias_por_setor = criar_vetor_distancias(angulos_rad, medidas, resolucao_angular)
densidade = construir_histograma(distancias_por_setor)
direcao = escolher_direcao(densidade, distancias_por_setor, largura_robo, objetivo_direcao=0, resolucao_angular=resolucao_angular)

print(f"Direção sugerida: {direcao}°")
plotar_comparativo(angulos_rad, medidas, distancias_por_setor, direcao, resolucao_angular)
