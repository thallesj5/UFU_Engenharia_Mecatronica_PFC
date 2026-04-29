//======================================================================

// Bibliotecas Utilizadas
#include <WiFi.h>
#include <PubSubClient.h>

//======================================================================

// Configurações de Rede e Tópicos
const char* password = "0507112533";
//Raspberry: 192.168.0.102 Note: 192.168.0.100
const char* mqtt_server = "192.168.0.102"; 
const char* ssid = "rede_pfc";

const int mqtt_port = 1883;
const char* mqtt_id = "ESP32_pfc";

//Tópicos MQTT
const char* mqtt_topico_controle = "pfc/controle";
const char* mqtt_topico_comando = "pfc/comando";
const char* mqtt_topico_config = "pfc/config";

//======================================================================

unsigned long last_msg_time = 0;
const unsigned long timeout_mqtt = 20000;

//======================================================================

// Pinos e PWM
const int ENA = 14; const int IN1 = 27; const int IN2 = 26; 
const int IN3 = 25; const int IN4 = 33; const int ENB = 32;
const int motor_esq = 0;
const int motor_dir = 1;
int v_esq = 0; 
int v_dir = 0;
int pwm_esq = 0;
int pwm_dir = 0;
unsigned long ms;
const int pwm_frequency = 1000;
const int pwm_resolution = 8;
float comp_dir = 0.99;

//======================================================================

// Modo Busca e rotação pura
const int ciclo_burst = 1000;
const int ciclo_rotacao = 100;
const int ligado_burst = 750;
int ligado_rotacao = 30;
int ligado_busca = 30;
const int pwm_busca = 210;       
int busca = 0;
unsigned long burst_ms;
int ultima_direcao = 0;

//======================================================================

WiFiClient espClient;
PubSubClient client(espClient);
void reconnect() {
  static unsigned long lastReconnectAttempt = 0;
  if (millis() - lastReconnectAttempt > 5000) {
    lastReconnectAttempt = millis();
    if (client.connect(mqtt_id)) {
      client.subscribe(mqtt_topico_controle);
      client.subscribe(mqtt_topico_comando);
      client.subscribe(mqtt_topico_config);
    }
  }
}

//======================================================================

void callback(char* topic, byte* payload, unsigned int length) {
  
  char msg[length + 1];
  memcpy(msg, payload, length);
  msg[length] = '\0';
  
  String topicoStr = String(topic);
  last_msg_time = millis(); // Reseta o timeout de segurança

  //Velocidade
  if (topicoStr == mqtt_topico_controle) {
    int v1, v2;
    // Velocidades: V_esq e V_dir
    int res = sscanf(msg, "%d,%d", &v1, &v2);
    if (res == 2) {
      v_esq = v1;
      v_dir = v2;
    }
    if (v_esq > v_dir) {
        ultima_direcao = 0; // Direita
      } 
    else if (v_dir > v_esq) {
      ultima_direcao = 1; // Esquerda
    }
  } 
  
  //Parâmetros de Configuração
  else if (topicoStr == mqtt_topico_config) {
    int r, b;
    //ligado_rotacao e ligado_busca)
    int res = sscanf(msg, "%d,%d", &r, &b);
    if (res == 2) {
      ligado_rotacao = r;
      ligado_busca = b;
    }
  }

//======================================================================


  // Ativa e Desativa Estado Buscar
  else if (topicoStr == mqtt_topico_comando) {
    busca = atoi(msg); 
  }
}

//======================================================================

// Comandos do Motor
void moveForward() { 
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW);  
  }
void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH); 
  digitalWrite(IN3, LOW);  
  digitalWrite(IN4, HIGH); 
  }
void turnLeft() { 
  digitalWrite(IN1, LOW);  
  digitalWrite(IN2, HIGH); 
  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW);  
  }
void turnRight() { 
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, LOW);  
  digitalWrite(IN4, HIGH); 
  }
void stopMotors() {
  digitalWrite(IN1, LOW);  
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, LOW);  
  digitalWrite(IN4, LOW);  
  }

//======================================================================

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT); 
  pinMode(IN4, OUTPUT);
  
  ledcSetup(motor_esq, pwm_frequency, pwm_resolution);
  ledcAttachPin(ENA, motor_esq);
  ledcSetup(motor_dir, pwm_frequency, pwm_resolution);
  ledcAttachPin(ENB, motor_dir);


//======================================================================


  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

//======================================================================

void loop() {
  if (!client.connected()) {
    reconnect();
  } else {
    client.loop();
  }

  // Timeout de segurança
  if (millis() - last_msg_time > timeout_mqtt) {
    v_esq = 0; v_dir = 0; busca = 0;
  }

  unsigned long ms = millis();
  pwm_esq = 0;
  pwm_dir = 0;

  // 1. Estado Buscar
  if (busca == 1) {
    burst_ms = ms % ciclo_burst;
    if (burst_ms < ligado_burst) {
      if (ultima_direcao == 1) {
        turnLeft();   
      } else {
        turnRight();  
      }
      if ((burst_ms % ciclo_rotacao) < ligado_busca) { 
        pwm_esq = pwm_busca; 
        pwm_dir = pwm_busca;
      } else { stopMotors(); }
    } else { stopMotors(); }
  } 
  
  // 2. Estado Parar
  else if (v_esq == 0 && v_dir == 0) {
    stopMotors();
    pwm_esq = 0;
    pwm_dir = 0;
  }

  // Movivemntação Transladar e Rotacionar
  else {
    // Estado Transladar: Avançar e Recuar
    if (v_esq == v_dir) {
      if (v_esq > 0) moveForward();
      else moveBackward();
      pwm_esq = abs(v_esq);
      pwm_dir = abs(v_dir);
    }
    // Estado Rotacionar: Virar Direita/Esquerda
    else {
      if (v_esq < v_dir) turnLeft(); 
      else turnRight();

      
      if ((ms % ciclo_rotacao) < ligado_rotacao) {
        pwm_esq = abs(v_esq); 
        pwm_dir = abs(v_dir);
      } else {
        stopMotors();
        pwm_esq = 0;
        pwm_dir = 0;
      }
    }
  }

  // Ajustes finais e PWM
  int final_pwm_esq = constrain(pwm_esq, 0, 255);
  int final_pwm_dir = constrain(round(pwm_dir * comp_dir), 0, 255);

  ledcWrite(motor_esq, final_pwm_esq);
  ledcWrite(motor_dir, final_pwm_dir);
}