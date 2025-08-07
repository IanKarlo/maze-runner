import csv
import math
import numpy as np
import matplotlib.pyplot as plt

largura_robo = 0.65              # Largura do robô em metros
resolucao_angular = 5            # Tamanho de cada setor (graus)
num_setores = 360 // resolucao_angular  # Total de setores

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
                if dist > 0:  # ignora leitura zero
                    angulos.append(ang_rad)  # radiano
                    distancias.append(dist)
            except ValueError:
                continue
    
    res = len(list(filter(lambda x: x < 0.1, distancias)))
    print(f'res: {res}')

    angulos = [math.pi + ang - math.pi/3 for ang in angulos]
    distancias = [0.3 if x <= 0 else x if x < 2 else 10 for x in distancias]
    return angulos, distancias

# === 2. Cria vetor de distâncias por setor ===
def criar_vetor_distancias(angulos_rad, medidas, resolucao_angular):
    num_setores = 360 // resolucao_angular
    distancias = np.ones(num_setores) * 10.0  # valor alto = sem obstáculo
    for ang_rad, dist in zip(angulos_rad, medidas):
        ang_deg = math.degrees(ang_rad) % 360
        indice = int(ang_deg // resolucao_angular)
        distancias[indice] = min(distancias[indice], dist)
    return distancias

# === 3. Histograma de densidade inversa ===
def construir_histograma(distancias, limiar=1.5):
    densidade = np.where(distancias < limiar, 1 / distancias, 0)
    return densidade

# === 4. Escolha da direção ideal considerando a largura do robô ===
def diferenca_angular(angle1, angle2):
    """Calcula a diferença mínima entre dois ângulos em graus no intervalo [-180, 180]."""
    diff = (angle1 - angle2 + 180) % 360 - 180
    return abs(diff)

def escolher_direcao(densidade, distancias, largura_robo=0.30, objetivo_direcao=0, resolucao_angular=10):
    num_setores = len(densidade)
    candidatos = []

    for i in range(num_setores):
        distancia = distancias[i]
        if distancia <= 0.1:
            continue
        try:
            angulo_necessario_rad = 2 * math.atan((largura_robo / 2) / distancia)
            abertura = int(math.degrees(angulo_necessario_rad) // resolucao_angular)
        except:
            continue

        abertura = max(abertura, 1)
        inicio = max(i - abertura // 2, 0)
        fim = min(i + abertura // 2 + 1, num_setores - 1)
        janela = densidade[inicio:fim]
        if np.sum(janela) < 1.0:
            candidatos.append(i)

    if not candidatos:
        return None

    melhor_indice = min(
        candidatos,
        key=lambda x: diferenca_angular(x * resolucao_angular, objetivo_direcao)
    )
    return melhor_indice * resolucao_angular


# === 5. Plotar gráficos lado a lado ===
def plotar_comparativo(angulos_rad, medidas, distancias_por_setor, direcao=None, resolucao_angular=5):
    angles_hist = np.radians(np.arange(0, 360, resolucao_angular))

    fig, axs = plt.subplots(1, 2, subplot_kw={'projection': 'polar'}, figsize=(12, 6))

    # Esquerda: Dispersão dos pontos
    axs[0].scatter(angulos_rad, medidas, s=10, c='green', alpha=0.7)
    axs[0].set_title("Dispersão das Leituras (LIDAR)")
    axs[0].set_theta_zero_location("N")  # 0° apontando para cima
    axs[0].set_theta_direction(-1)        # sentido anti-horário
    axs[0].set_rlim(0, 2.0) 

    # Direita: Histograma polar com setores visíveis
    larguras = np.radians(resolucao_angular)
    angles = np.radians(np.arange(0, 360, resolucao_angular))
    axs[1].bar(angles, distancias_por_setor, width=larguras, bottom=0.0, alpha=0.6, edgecolor='black', align='edge', color='skyblue')

    # Marca a direção sugerida (se houver)
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
