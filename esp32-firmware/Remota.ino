#include <WiFi.h>
#include <ModbusIP_ESP8266.h>
#include <ModbusRTU.h>
#include <HX711.h>
#include <ESP32Servo.h>
#include <driver/uart.h>

// ==========================================
// DEFINIÇÕES DE PINOS
// ==========================================
#define PIN_RX2     16
#define PIN_TX2     17
#define PIN_ENCODER 34
#define CHAVE_A     32
#define CHAVE_B     33
#define PIN_SERVO   13

// Pinos Ponte H L298N (Mini Ponte H MX1508)
#define IN1 26 // Borne OUT1 -> VCC do Motor de Vibração (Pulso Anti-Stall)
#define IN2 27 // Borne OUT2 -> Motor da Esteira no sentido invertido
#define IN3 14 // Motor de Giro Lateral
#define IN4 12 // Motor de Giro Lateral

// ==========================================
// PARÂMETROS DE CALIBRAÇÃO E FILTRAGEM
// ==========================================
#define FATOR_ESCALA_SILO   762.99f
#define FATOR_ESCALA_CAIXAA 767.56f
#define FATOR_ESCALA_CAIXAB 673.65f

#define ANGULO_VALVULA_FECHADA 40
#define ANGULO_VALVULA_ABERTA  138

#define ALPHA_CAIXA_NORMAL  0.65f
#define ALPHA_CAIXA_FINO    0.90f

ModbusIP mb_tcp;
ModbusRTU mb_rtu;
Servo servoValvula;

portMUX_TYPE hx711_mux = portMUX_INITIALIZER_UNLOCKED;

// ==========================================
// ESTRUTURA DE PROCESSAMENTO DE SINAL (DSP)
// ==========================================
struct CanalPesagem {
  HX711 sensor;
  uint8_t pin_dout;
  uint8_t pin_pd_sck;
  float fator_escala;
  float offset_tara_g;
  bool modo_rapido;
  volatile bool modo_fino;
  float buffer_mediana[3];
  bool inicializado;
  float peso_saida_g;
  volatile uint16_t peso_modbus_dg;

  void configurar(uint8_t pin_dt, uint8_t pin_sck, float escala, bool rapido) {
    pin_dout = pin_dt;
    pin_pd_sck = pin_sck;
    sensor.begin(pin_dt, pin_sck);
    fator_escala = (escala != 0.0f) ? escala : 1.0f;
    offset_tara_g = 0.0f;
    modo_rapido = rapido;
    modo_fino = false;
    inicializado = false;
    peso_saida_g = 0.0f;
    peso_modbus_dg = 0;
  }

  float calcularMediana3() {
    float a = buffer_mediana[0];
    float b = buffer_mediana[1];
    float c = buffer_mediana[2];
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
  }

  void atualizarSePronto() {
    if (digitalRead(pin_dout) != LOW) return;

    portENTER_CRITICAL(&hx711_mux);
    long leitura_bruta = sensor.read();
    portEXIT_CRITICAL(&hx711_mux);

    float peso_bruto_g = (float)leitura_bruta / fator_escala;

    if (!inicializado) {
      buffer_mediana[0] = buffer_mediana[1] = buffer_mediana[2] = peso_bruto_g;
      peso_saida_g = peso_bruto_g;
      offset_tara_g = peso_bruto_g;
      inicializado = true;
      return;
    }

    buffer_mediana[2] = buffer_mediana[1];
    buffer_mediana[1] = buffer_mediana[0];
    buffer_mediana[0] = peso_bruto_g;

    if (modo_rapido) {
      peso_saida_g = peso_bruto_g;
    } else {
      float peso_med = calcularMediana3();
      float alpha = modo_fino ? ALPHA_CAIXA_FINO : ALPHA_CAIXA_NORMAL;
      peso_saida_g = (alpha * peso_med) + ((1.0f - alpha) * peso_saida_g);
    }

    float peso_liquido_g = peso_saida_g - offset_tara_g;
    if (peso_liquido_g < 0.0f) peso_liquido_g = 0.0f;
    if (peso_liquido_g > 6500.0f) peso_liquido_g = 6500.0f;

    peso_modbus_dg = (uint16_t)(peso_liquido_g * 10.0f + 0.5f);
  }

  void tararInstantaneo() {
    offset_tara_g = peso_saida_g;
    peso_modbus_dg = 0;
  }
};

CanalPesagem balancaSilo;
CanalPesagem balancaCaixaA;
CanalPesagem balancaCaixaB;

volatile bool req_tara_silo   = false;
volatile bool req_tara_caixaA = false;
volatile bool req_tara_caixaB = false;

unsigned long tempo_ultimo_pacote = 0;
uint16_t wd_anterior = 0;
uint16_t wd_esp32    = 0;
unsigned long timer_wd_esp = 0;

bool usar_tcp = false;
uint16_t ultimo_cmd_rtu = 0;
uint16_t ultimo_cmd_tcp = 0;

unsigned long timer_antistall  = 0;
unsigned long timer_fim_pulso  = 0;
unsigned long timer_janela_vib = 0;
uint16_t peso_silo_anterior_dg = 0;
bool vibrador_pulsando         = false;

void TaskPesagem(void *pvParameters) {
  for (;;) {
    if (req_tara_silo)   { balancaSilo.tararInstantaneo();   req_tara_silo = false; }
    if (req_tara_caixaA) { balancaCaixaA.tararInstantaneo(); req_tara_caixaA = false; }
    if (req_tara_caixaB) { balancaCaixaB.tararInstantaneo(); req_tara_caixaB = false; }

    balancaSilo.atualizarSePronto();
    balancaCaixaA.atualizarSePronto();
    balancaCaixaB.atualizarSePronto();

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

Modbus::ResultCode cbMonitorModbus(Modbus::FunctionCode fc, const Modbus::RequestData data) {
  delay(2);
  return Modbus::EX_SUCCESS;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(IN1, OUTPUT); digitalWrite(IN1, LOW);
  pinMode(IN2, OUTPUT); digitalWrite(IN2, LOW);
  pinMode(IN3, OUTPUT); digitalWrite(IN3, LOW);
  pinMode(IN4, OUTPUT); digitalWrite(IN4, LOW);

  pinMode(CHAVE_A, INPUT_PULLUP);
  pinMode(CHAVE_B, INPUT_PULLUP);

  servoValvula.attach(PIN_SERVO);
  servoValvula.write(ANGULO_VALVULA_FECHADA);

  balancaSilo.configurar(19, 18, FATOR_ESCALA_SILO, true);
  balancaCaixaA.configurar(21, 22, FATOR_ESCALA_CAIXAA, false);
  balancaCaixaB.configurar(23, 25, FATOR_ESCALA_CAIXAB, false);

  Serial2.begin(19200, SERIAL_8N2, PIN_RX2, PIN_TX2);
  uart_set_tx_idle_num(UART_NUM_2, 4);

  mb_rtu.begin(&Serial2);
  mb_rtu.slave(1);
  mb_rtu.onRequest(cbMonitorModbus);

  WiFi.mode(WIFI_STA);
  WiFi.begin("PEDRO", "cde02d8777d8adad278c11dada1beee0378b97bdd3c65b3aecb464dd5bd24730");
  unsigned long t_inicio_wifi = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t_inicio_wifi < 5000)) {
    delay(250);
  }

  mb_tcp.server();
  for (int i = 0; i <= 16; i++) {
    mb_tcp.addHreg(i, 0);
    mb_rtu.addHreg(i, 0);
  }

  xTaskCreatePinnedToCore(TaskPesagem, "TaskPesagem", 4096, NULL, 2, NULL, 1);
}

void loop() {
  mb_tcp.task();
  mb_rtu.task();
  Serial2.flush();

  // 1. CHAVEAMENTO DE REDE (REG 16)
  uint16_t cmd_rtu = mb_rtu.Hreg(16);
  uint16_t cmd_tcp = mb_tcp.Hreg(16);

  if (cmd_rtu != ultimo_cmd_rtu) {
    usar_tcp = (cmd_rtu == 1);
    ultimo_cmd_rtu = cmd_rtu;
    mb_tcp.Hreg(16, cmd_rtu);
    ultimo_cmd_tcp = cmd_rtu;
  } else if (cmd_tcp != ultimo_cmd_tcp) {
    usar_tcp = (cmd_tcp == 1);
    ultimo_cmd_tcp = cmd_tcp;
    mb_rtu.Hreg(16, cmd_tcp);
    ultimo_cmd_rtu = cmd_tcp;
  }

  // 2. WATCHDOG SELETIVO (500 ms)
  uint16_t wd_atual = usar_tcp ? mb_tcp.Hreg(15) : mb_rtu.Hreg(15);
  if (wd_atual != wd_anterior) {
    tempo_ultimo_pacote = millis();
    wd_anterior = wd_atual;
  }

  if (millis() - timer_wd_esp >= 100) {
    wd_esp32 = (wd_esp32 + 1) % 1000;
    mb_rtu.Hreg(5, wd_esp32);
    mb_tcp.Hreg(5, wd_esp32);
    timer_wd_esp = millis();
  }

  bool falha_rede = (millis() - tempo_ultimo_pacote > 500);

  // 3. ESPELHAMENTO HONESTO E DIRETO DOS SENSORES (REG 0 a 4)
  uint16_t pSilo = balancaSilo.peso_modbus_dg;
  uint16_t pCxA  = balancaCaixaA.peso_modbus_dg;
  uint16_t pCxB  = balancaCaixaB.peso_modbus_dg;

  if (!vibrador_pulsando && millis() >= timer_janela_vib) {
    mb_rtu.Hreg(0, pSilo);
    mb_tcp.Hreg(0, pSilo);
  }
  mb_rtu.Hreg(1, pCxA); mb_tcp.Hreg(1, pCxA);
  mb_rtu.Hreg(2, pCxB); mb_tcp.Hreg(2, pCxB);

  uint16_t statusChaves = 0;
  bool limiteA = (digitalRead(CHAVE_A) == LOW);
  bool limiteB = (digitalRead(CHAVE_B) == LOW);
  if (limiteA) bitSet(statusChaves, 0);
  if (limiteB) bitSet(statusChaves, 1);
  mb_rtu.Hreg(4, statusChaves);
  mb_tcp.Hreg(4, statusChaves);

  // 4. ATUAÇÃO, FILTRO ADAPTATIVO E ANTI-STALL
  uint16_t cmdTara = usar_tcp ? mb_tcp.Hreg(14) : mb_rtu.Hreg(14);
  if (cmdTara > 0) {
    if (bitRead(cmdTara, 0)) req_tara_silo   = true;
    if (bitRead(cmdTara, 1)) req_tara_caixaA = true;
    if (bitRead(cmdTara, 2)) req_tara_caixaB = true;
    mb_rtu.Hreg(14, 0);
    mb_tcp.Hreg(14, 0);
  }

  uint16_t pctValvula = usar_tcp ? mb_tcp.Hreg(10) : mb_rtu.Hreg(10);
  uint16_t pwmEsteira = usar_tcp ? mb_tcp.Hreg(11) : mb_rtu.Hreg(11);
  uint16_t pwmN20     = usar_tcp ? mb_tcp.Hreg(12) : mb_rtu.Hreg(12);
  uint16_t dirN20     = usar_tcp ? mb_tcp.Hreg(13) : mb_rtu.Hreg(13);

  if (pctValvula > 100) pctValvula = 100;
  if (pwmEsteira > 255) pwmEsteira = 255;
  if (pwmN20 > 255)     pwmN20     = 255;

  bool ativar_fino = (pwmEsteira <= 200);
  balancaCaixaA.modo_fino = ativar_fino;
  balancaCaixaB.modo_fino = ativar_fino;

  int anguloServo = map(pctValvula, 0, 100, ANGULO_VALVULA_FECHADA, ANGULO_VALVULA_ABERTA);
  servoValvula.write(anguloServo);

  unsigned long agora = millis();
  static uint16_t valvula_anterior = 0;
  if (pctValvula > 0 && valvula_anterior == 0) {
    timer_antistall = agora;
    peso_silo_anterior_dg = pSilo;
  }
  valvula_anterior = pctValvula;

  unsigned long tempo_espera_stall = (pctValvula < 90) ? 700 : 850;
  unsigned long duracao_pulso_vib  = 160;

  if (pctValvula > 0 && pSilo > 10) {
    if (vibrador_pulsando) {
      if (agora >= timer_fim_pulso) {
        analogWrite(IN1, 0);
        vibrador_pulsando = false;
        timer_janela_vib  = agora + 60;
        timer_antistall   = agora;
        peso_silo_anterior_dg = pSilo;
      }
    } else {
      if ((int)peso_silo_anterior_dg - (int)pSilo >= 2) {
        peso_silo_anterior_dg = pSilo;
        timer_antistall = agora;
      } else if (agora - timer_antistall >= tempo_espera_stall) {
        analogWrite(IN2, 0);
        analogWrite(IN1, 255);
        vibrador_pulsando = true;
        timer_fim_pulso   = agora + duracao_pulso_vib;
      }
    }
  } else {
    analogWrite(IN1, 0);
    vibrador_pulsando = false;
    timer_antistall = agora;
    peso_silo_anterior_dg = pSilo;
  }

  if (pwmEsteira > 0 && !vibrador_pulsando) {
    analogWrite(IN2, pwmEsteira);
  } else {
    analogWrite(IN2, 0);
  }

  // Em atraso de rede corta apenas o Giro Lateral; válvula e esteira obedecem ao último comando do CLP
  if (falha_rede) {
    analogWrite(IN3, 0);
    analogWrite(IN4, 0);
  } else {
    bool comandoAvancaA = (pwmN20 > 0 && dirN20 == 0);
    bool comandoAvancaB = (pwmN20 > 0 && dirN20 == 1);

    if (comandoAvancaA && limiteA) comandoAvancaA = false;
    if (comandoAvancaB && limiteB) comandoAvancaB = false;

    if (comandoAvancaA) {
      analogWrite(IN3, pwmN20);
      analogWrite(IN4, 0);
    } else if (comandoAvancaB) {
      analogWrite(IN3, 0);
      analogWrite(IN4, pwmN20);
    } else {
      analogWrite(IN3, 0);
      analogWrite(IN4, 0);
    }
  }
}
