# PFC - Engenharia Mecatrônica UFU
IMPLEMENTAÇÃO DE ROBÔ MÓVEL PARA RASTREIO DE PESSOA COM VISÃO COMPUTACIONAL

Este repositório contém a documentação técnica, firmwares e scripts desenvolvidos para o Projeto de Fim de Curso (PFC) em Engenharia Mecatrônica na **Universidade Federal de Uberlândia (UFU)**. O projeto foca na integração de visão computacional para rastreio de uma pessoa.

=============================================================================================================

## Estrutura do Repositório

**/RaspberryPi**: Script principal (`final.py`) responsável pela detecção via YOLO, filtragem de trajetória com Filtro de Kalman e comunicação via protocolo MQTT.

**/ESP32**: Firmware (`esp32.ino`) em C++ para controle dos motores e execução dos comandos de movimento recebidos via ESP32 e comunicação MQTT.

=============================================================================================================

## Configuração do Sistema Operacional (Raspberry Pi)

Para garantir a performance da visão computacional, o sistema foi configurado conforme os passos abaixo:

### 1. Dependências do Sistema e Broker MQTT
Instalação de ferramentas de compilação, bibliotecas gráficas e o broker MQTT Mosquitto:
bash:
sudo apt update && sudo apt upgrade -y && \
sudo apt install -y build-essential libssl-dev zlib1g-dev libncurses5-dev \
libncursesw5-dev libreadline-dev libsqlite3-dev tk-dev libgdbm-dev \
libc6-dev libbz2-dev libffi-dev xz-utils libgl1 libglx-mesa0 libglib2.0-0 \
mosquitto mosquitto-clients

### 2. Configuração do Broker Local

echo -e "listener 1883 0.0.0.0\nallow_anonymous true" | sudo tee /etc/mosquitto/conf.d/local.conf
sudo systemctl restart mosquitto

### 3. Versão do Python

cd /tmp
wget [https://www.python.org/ftp/python/3.12.3/Python-3.12.3.tar.xz](https://www.python.org/ftp/python/3.12.3/Python-3.12.3.tar.xz)
tar -xf Python-3.12.3.tar.xz
cd Python-3.12.3
./configure --enable-optimizations
make -j$(nproc)
sudo make altinstall

=============================================================================================================

## Conectividade e Acesso Remoto


bash:
sudo nano /etc/wpa_supplicant/wpa_supplicant.conf

Editar para:

ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=BR

network={
    ssid="rede_pfc"
    psk="0507112533"
    priority=10
}

=============================================================================================================

## Ambiente virtual:

### Criação do ambiente virtual
python3.12 -m venv pfc
source pfc/bin/activate

### Instalação das dependências com versões travadas
pip install --upgrade pip
pip install lapx
pip install numpy==2.2.6 opencv-python==4.12.0.88 ultralytics==8.3.202 filterpy==1.4.5 paho-mqtt==2.1.0

