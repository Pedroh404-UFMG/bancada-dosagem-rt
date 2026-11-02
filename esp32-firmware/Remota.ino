#include <WiFi.h>
#include <ModbusIP_ESP8266.h>
#include <ModbusRTU.h>
#include <HX711.h>
#include <ESP32Servo.h>

// ==========================================
// DEFINIÇÕES DE PINOS (MANTIDOS INTACTOS)
// ==========================================
#define PIN_RX2     16
#define PIN_TX2     17
#define PIN_ENCODER 34
#define CHAVE_A     32
#define CHAVE_B     33
#define PIN_SERVO   13

// Pinos Ponte H L298N
#define IN1 26 // Tração da Cinta da Esteira (Motor N20)
#define IN2 27 // Tração da Cinta da Esteira (Motor N20)
#define IN3 14 // Movimento Translacional / Giro para Caixas (Motor TT)
#define IN4 12 // Movimento Translacional / Giro para Caixas (Motor TT)

// ==========================================
// PARÂMETROS DE CALIBRAÇÃO E FILTRAGEM ASSIMÉTRICA
// ==========================================
// Fatores de calibração individuais (contagens brutas por grama)
#define FATOR_ESCALA_SILO   420.0f 
#define FATOR_ESCALA_CAIXAA 420.0f
#define FATOR_ESCALA_CAIXAB 365.5f // Ajustado para compensar sensibilidade da célula B

// Limites físicos do servomotor da comporta guilhotina (em graus: 0 a 180)
#define ANGULO_VALVULA_FECHADA 0
#define ANGULO_VALVULA_ABERTA  90

// Parâmetros da Cascata DSP das Caixas Receptoras (Amortecimento de Impacto h_q = 9,0 cm)
#define MAX_DERIVADA_CAIXA  2.5f  // Limite máximo de variação (g/amostra) contra impacto
#define ALPHA_CAIXA         0.25f // Filtro Passa-Baixas 1ª Ordem: H(z) = 0.25 / (1 - 0.75 z^-1)

ModbusIP mb_tcp;
ModbusRTU mb_rtu;
Servo servoValvula;

// Spinlock para proteger os ~25us de deslocamento de bits do HX711
portMUX_TYPE hx711_mux = portMUX_INITIALIZER_UNLOCKED;

// ==========================================
// ESTRUTURA DE PROCESSAMENTO DE SINAL (DSP ASSIMÉTRICO)
// ==========================================
struct CanalPesagem {
  HX711 sensor;
  uint8_t pin_dout;
  uint8_t pin_pd_sck;
  float fator_escala;
  float offset_tara_g;
  bool modo_rapido;       // true = Silo (Atraso Zero) | false = Caixas (Cascata 3 Estágios)
  uint8_t tam_janela;     // N = 3 para Silo | N = 5 para Caixas
  float buffer_mediana[5];
  uint8_t idx_mediana;
  bool inicializado;
  float peso_saida_g;     // Saída atual em gramas (direta no Silo / filtrada IIR nas Caixas)
  float max_delta_g;
  float alpha_iir;
  volatile uint16_t peso_modbus_dg; // Peso líquido em DECIGRAMAS (0,1 g) para o Modbus

  void configurar(uint8_t pin_dt, uint8_t pin_sck, float escala, bool rapido, float max_d, float alpha) {
    pin_dout = pin_dt;
    pin_pd_sck = pin_sck;
    sensor.begin(pin_dt, pin_sck);
    fator_escala = (escala != 0.0f) ? escala : 1.0f;
    offset_tara_g = 0.0f;
    modo_rapido = rapido;
    tam_janela = rapido ? 3 : 5;
    idx_mediana = 0;
    inicializado = false;
    peso_saida_g = 0.0f;
    max_delta_g = max_d;
    alpha_iir = alpha;
    peso_modbus_dg = 0;
  }

  // Filtro de Mediana configurável (N=3 para Silo rápido, N=5 para Caixas)
  float calcularMediana() {
    float temp[5];
    uint8_t n = tam_janela;
    for (uint8_t i = 0; i < n; i++) temp[i] = buffer_mediana[i];
    for (uint8_t i = 0; i < n - 1; i++) {
      for (uint8_t j = i + 1; j < n; j++) {
        if (temp[j] < temp[i]) {
          float t = temp[i];
          temp[i] = temp[j];
          temp[j] = t;
        }
      }
    }
    return temp[n / 2]; // Retorna elemento central
  }

  // Aquisição não-bloqueante e processamento assimétrico
  void atualizarSePronto() {
    // Verifica diretamente o pino físico DOUT sem bloquear a CPU
    if (digitalRead(pin_dout) != LOW) return;

    // Seção crítica ultrarrápida (~25 us) apenas para os 24 pulsos de clock
    portENTER_CRITICAL(&hx711_mux);
    long leitura_bruta = sensor.read();
    portEXIT_CRITICAL(&hx711_mux);

    float peso_bruto_g = (float)leitura_bruta / fator_escala;

    if (!inicializado) {
      for (int i = 0; i < 5; i++) buffer_mediana[i] = peso_bruto_g;
      peso_saida_g = peso_bruto_g;
      offset_tara_g = peso_bruto_g; // Auto-tara inicial ao energizar
      inicializado = true;
      return;
    }

    // 1º Estágio: Descarte de spikes elétricos via Mediana (N=3 no Silo, N=5 nas Caixas)
    buffer_mediana[idx_mediana] = peso_bruto_g;
    idx_mediana = (idx_mediana + 1) % tam_janela;
    float peso_mediana = calcularMediana();

    if (modo_rapido) {
      // RAMO 1 (SILO DOSADOR): Atraso de fase ZERO (H(z) = 1, tau_f = 0 ms)
      // Essencial para detectar instantaneamente os 50g sob alta vazão e fechar a válvula
      peso_saida_g = peso_mediana;
    } else {
      // RAMO 2 (CAIXAS RECEPTORAS A e B): Cascata contra impacto de queda livre (9,0 cm)
      // 2º Estágio: Limitador Não-Linear de Derivada (Slew-Rate Limiter)
      float delta = peso_mediana - peso_saida_g;
      if (delta > max_delta_g) delta = max_delta_g;
      else if (delta < -max_delta_g) delta = -max_delta_g;
      float peso_lim_g = peso_saida_g + delta;

      // 3º Estágio: Filtro Passa-Baixas Discreto de 1ª Ordem (Equação a diferenças IIR)
      peso_saida_g = (alpha_iir * peso_lim_g) + ((1.0f - alpha_iir) * peso_saida_g);
    }

    // Subtração de Tara e Quantização em Ponto Fixo na escala de Decigramas (0,1 g)
    float peso_liquido_g = peso_saida_g - offset_tara_g;
    if (peso_liquido_g < 0.0f) peso_liquido_g = 0.0f; // Proteção contra underflow no uint16_t
    if (peso_liquido_g > 6500.0f) peso_liquido_g = 6500.0f;

    peso_modbus_dg = (uint16_t)(peso_liquido_g * 10.0f + 0.5f);
  }

  // Tara algébrica instantânea (0 ms de bloqueio)
  void tararInstantaneo() {
    offset_tara_g = peso_saida_g;
    peso_modbus_dg = 0;
  }
};

CanalPesagem balancaSilo;
CanalPesagem balancaCaixaA;
CanalPesagem balancaCaixaB;

// Flags de comando de tara
volatile bool req_tara_silo   = false;
volatile bool req_tara_caixaA = false;
volatile bool req_tara_caixaB = false;

// Variáveis de Controle e Watchdog (500 ms)
unsigned long tempo_ultimo_pacote = 0;
uint16_t wd_anterior = 0;
uint16_t wd_esp32    = 0;
unsigned long timer_wd_esp = 0;

// Variáveis de Comutação de Rede (Reg 16: 0 = Modbus RTU, 1 = Modbus TCP)
bool usar_tcp = false; // Inicia em Modbus RTU (RS-485) por padrão
uint16_t ultimo_cmd_rtu = 0;
uint16_t ultimo_cmd_tcp = 0;

// ==========================================
// TAREFA FREERTOS DEDICADA À PESAGEM (CORE 1)
// ==========================================
void TaskPesagem(void *pvParameters) {
  for (;;) {
    if (req_tara_silo)   { balancaSilo.tararInstantaneo();   req_tara_silo = false; }
    if (req_tara_caixaA) { balancaCaixaA.tararInstantaneo(); req_tara_caixaA = false; }
    if (req_tara_caixaB) { balancaCaixaB.tararInstantaneo(); req_tara_caixaB = false; }

    balancaSilo.atualizarSePronto();
    balancaCaixaA.atualizarSePronto();
    balancaCaixaB.atualizarSePronto();

    vTaskDelay(pdMS_TO_TICKS(2)); // Verifica disponibilidade a cada 2 ms (taxa HX711 = 12,5 ms / 100 ms)
  }
}

void setup() {
  // 1. TRAVA IMEDIATA DA PONTE H NO BOOT (Evita disparo por pull-up JTAG nos pinos 12 e 14)
  pinMode(IN1, OUTPUT); digitalWrite(IN1, LOW);
  pinMode(IN2, OUTPUT); digitalWrite(IN2, LOW);
  pinMode(IN3, OUTPUT); digitalWrite(IN3, LOW);
  pinMode(IN4, OUTPUT); digitalWrite(IN4, LOW);

  pinMode(CHAVE_A, INPUT_PULLUP);
  pinMode(CHAVE_B, INPUT_PULLUP);

  // Inicializa servomotor na posição fechada antes do bloqueio de rede Wi-Fi
  servoValvula.attach(PIN_SERVO);
  servoValvula.write(ANGULO_VALVULA_FECHADA);

  Serial.begin(115200);

  // 2. Configura os 3 canais de pesagem (Silo em modo rápido; Caixas A e B com cascata DSP)
  balancaSilo.configurar(19, 18, FATOR_ESCALA_SILO, true, 0.0f, 1.0f);
  balancaCaixaA.configurar(21, 22, FATOR_ESCALA_CAIXAA, false, MAX_DERIVADA_CAIXA, ALPHA_CAIXA);
  balancaCaixaB.configurar(23, 25, FATOR_ESCALA_CAIXAB, false, MAX_DERIVADA_CAIXA, ALPHA_CAIXA);

  // 3. Inicializa barramento serial RS-485 (Modbus RTU Slave ID 1)
  Serial2.begin(19200, SERIAL_8N1, PIN_RX2, PIN_TX2);
  mb_rtu.begin(&Serial2);
  mb_rtu.slave(1);

  // 4. Conecta ao Wi-Fi com timeout de segurança (não trava o RS-485 caso o roteador esteja desligado)
  WiFi.mode(WIFI_STA);
  WiFi.begin("PEDRO", "cde02d8777d8adad278c11dada1beee0378b97bdd3c65b3aecb464dd5bd24730");
  unsigned long t_inicio_wifi = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t_inicio_wifi < 8000)) {
    delay(250);
  }
  mb_tcp.server();

  // Aloca o Mapa Unificado de Registradores (Holding Registers 0 a 16)
  for (int i = 0; i <= 16; i++) {
    mb_tcp.addHreg(i, 0);
    mb_rtu.addHreg(i, 0);
  }

  // 5. Fixa a TaskPesagem no Core 1 (deixando o Core 0 livre para interrupções do rádio Wi-Fi)
  xTaskCreatePinnedToCore(TaskPesagem, "TaskPesagem", 4096, NULL, 2, NULL, 1);

  tempo_ultimo_pacote = millis();
}

void loop() {
  mb_tcp.task();
  mb_rtu.task();

  // ==========================================
  // 1. CHAVEAMENTO DE REDE EM TEMPO DE EXECUÇÃO (REG 16)
  // ==========================================
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

  // ==========================================
  // 2. WATCHDOG SELETIVO DA REDE ATIVA (500 ms)
  // ==========================================
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

  // ==========================================
  // 3. ESPELHAMENTO IMEDIATO DOS SENSORES (REG 0 a 4)
  // ==========================================
  uint16_t pSilo = balancaSilo.peso_modbus_dg;
  uint16_t pCxA  = balancaCaixaA.peso_modbus_dg;
  uint16_t pCxB  = balancaCaixaB.peso_modbus_dg;

  mb_rtu.Hreg(0, pSilo); mb_tcp.Hreg(0, pSilo);
  mb_rtu.Hreg(1, pCxA);  mb_tcp.Hreg(1, pCxA);
  mb_rtu.Hreg(2, pCxB);  mb_tcp.Hreg(2, pCxB);

  uint16_t statusChaves = 0;
  bool limiteA = (digitalRead(CHAVE_A) == LOW);
  bool limiteB = (digitalRead(CHAVE_B) == LOW);
  if (limiteA) bitSet(statusChaves, 0);
  if (limiteB) bitSet(statusChaves, 1);
  mb_rtu.Hreg(4, statusChaves);
  mb_tcp.Hreg(4, statusChaves);

  // ==========================================
  // 4. ATUAÇÃO E INTERTRAVAMENTOS DE SEGURANÇA (REG 10 a 14)
  // ==========================================
  uint16_t cmdTara = usar_tcp ? mb_tcp.Hreg(14) : mb_rtu.Hreg(14);
  if (cmdTara > 0) {
    if (bitRead(cmdTara, 0)) req_tara_silo   = true;
    if (bitRead(cmdTara, 1)) req_tara_caixaA = true;
    if (bitRead(cmdTara, 2)) req_tara_caixaB = true;
    mb_rtu.Hreg(14, 0);
    mb_tcp.Hreg(14, 0);
  }

  if (falha_rede) {
    // Watchdog Seletivo: corta giro lateral (N20) para proteger a estrutura,
    // mas retém comporta e esteira para evidenciar o erro físico de batelada!
    analogWrite(IN3, 0);
    analogWrite(IN4, 0);
  } else {
    uint16_t pctValvula = usar_tcp ? mb_tcp.Hreg(10) : mb_rtu.Hreg(10);
    uint16_t pwmEsteira = usar_tcp ? mb_tcp.Hreg(11) : mb_rtu.Hreg(11);
    uint16_t pwmN20     = usar_tcp ? mb_tcp.Hreg(12) : mb_rtu.Hreg(12);
    uint16_t dirN20     = usar_tcp ? mb_tcp.Hreg(13) : mb_rtu.Hreg(13);

    if (pctValvula > 100) pctValvula = 100;
    if (pwmEsteira > 255) pwmEsteira = 255;
    if (pwmN20 > 255)     pwmN20     = 255;

    // Aciona comporta guilhotina respeitando os limites angulares calibrados
    int anguloServo = map(pctValvula, 0, 100, ANGULO_VALVULA_FECHADA, ANGULO_VALVULA_ABERTA);
    servoValvula.write(anguloServo);

    // Acionamento do motor da esteira (TT)
    if (pwmEsteira > 0) {
      analogWrite(IN1, pwmEsteira);
      analogWrite(IN2, 0);
    } else {
      analogWrite(IN1, 0);
      analogWrite(IN2, 0);
    }

    // Acionamento do micromotor de giro (N20) com intertravamento local por Fim de Curso
    // dirN20 == 0 -> Move para Caixa A | dirN20 == 1 -> Move para Caixa B
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
