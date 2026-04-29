# ==============================================================
# Lista de Bibliotecas
import cv2
from ultralytics import YOLO
import numpy as np
import time
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import paho.mqtt.client as mqtt
import csv

# ==============================================================
# Gráficos

arquivo_csv = "dados2aaaaaaaaaaaaaaaaaaaaaaaaaa.csv"
f = open(arquivo_csv, mode='w', newline='')
writer = csv.writer(f)
writer.writerow(['tempo', 'x_yolo', 'x_kalman', 'h_yolo', 'h_kalman', 'dt', 'busca'])

x_norm = 0
x_filtrado = 0
h_norm = 0
h_filtrado = 0
tempo_exp = 0

# ==============================================================
# Topicos mqtt
mqtt_broker = "localhost" 
mqtt_id = "Raspberry_pfc"
mqtt_topico_comando = "pfc/comandos"
mqtt_topico_controle = "pfc/controle"
mqtt_topico_config = "pfc/config"

# ==============================================================
# Dimensões da Tela
LARGURA_TELA = 320
ALTURA_TELA = 240
# Variaveis de Controle
h_min = 0.8 
h_max = 0.9 
DEADBAND_ERRO = 0.1
v_max = 255
v_esq = 0
v_dir = 0
v_base = 200 #210
vel_rotacao = 30 #30
vel_busca = 30

# ==============================================================
# Variaveis de Loop
FRAMES_PARA_PULAR = 3
alvo_id = None
frame_counter = 0
tempo_inferencia = 0
resultados = None


# ==============================================================
# Modo Buscar
LIMITE_BUSCA = 40 #40
perdas_consecutivas = 0
busca = 0

# ==============================================================
# MQTT
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Conectado ao Broker!")
        client.publish(mqtt_topico_config, f"{vel_rotacao},{vel_busca}", retain=True, qos=1)
    else:
        print(f"Falha na conexão, código: {rc}")

# Cliente MQTT
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id=mqtt_id)
client.on_connect = on_connect
client.connect(mqtt_broker, 1883, 60)
client.loop_start()

# ==============================================================
# Camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, LARGURA_TELA)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, ALTURA_TELA)

# Yolo
model = YOLO('yolo11n.pt')

# ==============================================================
# Configuração Filto de Kalman
dt = 0.033

# Posição x
kf_x = KalmanFilter(dim_x=2, dim_z=1)
kf_x.x = np.array([[0.], 
                   [0.]])
kf_x.F = np.array([[1., dt], 
                   [0., 1.]])
kf_x.H = np.array([[1., 0.]])
kf_x.P = np.eye(2) * 10.
kf_x.R = np.array([[0.36e-5]])
sigma_ax = 1.5
kf_x.Q = sigma_ax**2 * np.array([
    [dt**4/4, dt**3/2],
    [dt**3/2, dt**2]
])

# Altura h
kf_h = KalmanFilter(dim_x=2, dim_z=1)
kf_h.x = np.array([[0.9], 
                   [0.]])
kf_h.F = np.array([[1., dt], 
                   [0., 1.]])
kf_h.H = np.array([[1., 0.]])
kf_h.P = np.eye(2) * 5.
kf_h.R = np.array([[59.29e-5]])
sigma_ah = 5
kf_h.Q = sigma_ah**2 * np.array([
    [dt**4/4, dt**3/2],
    [dt**3/2, dt**2]
])

time.sleep(4)
try:
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, LARGURA_TELA)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, ALTURA_TELA)
        width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        success, frame = cap.read()
        print(f"Frame width set to: {width}, Frame height set to: {height}")

    tk_0 = time.time()
    tk_1 = 0
    
    while cap.isOpened():
        tk_utc = time.time()
        tk = tk_utc - tk_0
        dt = tk - tk_1
        tk_1 = tk
        success, frame = cap.read()
        if not success:
            break
        frame = cv2.resize(frame, (320, 240))

        # Cálculo Dinâmico

        kf_x.F[0, 1] = dt
        kf_x.Q = sigma_ax**2 * np.array([
            [dt**4/4, dt**3/2],
            [dt**3/2, dt**2]
        ])
        kf_x.predict()

        kf_h.F[0, 1] = dt
        kf_h.Q = sigma_ah**2 * np.array([
            [dt**4/4, dt**3/2],
            [dt**3/2, dt**2]
        ])
        kf_h.predict()

        
        alvo_encontrado_neste_frame = False

        # Processamento YOLO
        if frame_counter % (FRAMES_PARA_PULAR + 1) == 0:
            resultados = model.track(frame, classes=0, persist=True, verbose=False, imgsz=320)
            
            if resultados[0].boxes.id is not None:
                ids_detectados = resultados[0].boxes.id.int().cpu().tolist()

                if alvo_id is None or (busca == 1):
                    alvo_id = ids_detectados[0]

                for i, track_id in enumerate(ids_detectados):
                    if track_id == alvo_id:
                        alvo_encontrado_neste_frame = True
                        perdas_consecutivas = 0
                        x_pixel, _, _, h_pixels = resultados[0].boxes.xywh[i]
                        busca = 0

                        # Normalização
                        x_norm = (x_pixel - LARGURA_TELA / 2) / LARGURA_TELA
                        h_norm = float(h_pixels) / ALTURA_TELA
                        # Atualização do Filtro de
                        kf_x.update(np.array([[x_norm]]))
                        kf_h.update(np.array([[h_norm]]))


        #Estado Buscar
        if not alvo_encontrado_neste_frame and alvo_id is not None:
            perdas_consecutivas += 1
            kf_x.update(np.array([[x_norm]]))
            kf_h.update(np.array([[h_norm]]))
            if perdas_consecutivas > LIMITE_BUSCA:
                busca = 1
                kf_x.update(np.array([[x_norm]]))
                kf_h.update(np.array([[h_norm]]))
                client.publish(mqtt_topico_comando, f"{busca}", qos=0)
                print("\nComando: Buscar")

        frame_counter += 1
        x_filtrado = kf_x.x[0].item()
        h_filtrado = kf_h.x[0].item()

        #Salvando itens nos gráficos
        writer.writerow([f"{tk:.4f}", f"{x_norm:.4f}", f"{x_filtrado:.4f}", f"{h_norm:.4f}", f"{h_filtrado:.4f}",f"{dt:.4f}", f"{busca}"])

        # FSM
        if busca == 0:
            if x_filtrado >= DEADBAND_ERRO:
                #print('Direita!\n')
                v_esq = v_base
                v_dir = -v_base
            elif x_filtrado <= -DEADBAND_ERRO:
                #print('Esquerda!\n')
                v_esq = -v_base
                v_dir = v_base
            elif h_filtrado > h_max:
                #print('Recua')
                v_esq = -v_base
                v_dir = -v_base
            elif h_filtrado <= h_min:
                #print('Avança!')
                v_esq = v_base
                v_dir = v_base
            else:
                #print('Parado')
                v_esq = 0
                v_dir = 0
                
            # Enviando ao MQTT
            client.publish(mqtt_topico_controle, f"{v_esq},{v_dir}", qos=0)

        # Enviando ao MQTT
        client.publish(mqtt_topico_comando, f"{busca}", qos=0)
        # VISUALIZAÇÃO
        print(f"tempo:{tempo_exp:.3f}, x_norm: {x_norm:.4f}, x_kalman: {x_filtrado:.4f}, h_norm: {h_norm:.4f}, h_kalman: {h_filtrado:.4f}, dt: {1000 * dt:.0f}, busca: {busca}")

except KeyboardInterrupt:
    print("\n[SISTEMA] Encerrando...")
    client.publish(mqtt_topico_controle, "0,0", qos=1)
    client.disconnect()
    time.sleep(2)
finally:
    cap.release()
    client.loop_stop()
    client.disconnect()