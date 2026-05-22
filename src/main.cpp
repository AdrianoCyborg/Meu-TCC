#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>

//////////////////////
// CONFIG WIFI E MQTT
//////////////////////
const char* ssid = "NOME_DA_TUA_REDE";
const char* password = "SENHA_DA_REDE";
const char* mqtt_server = "192.168.15.5"; 

WiFiClient espClient;
PubSubClient client(espClient);

//////////////////////
// PINOS DE HARDWARE
//////////////////////
#define PINO_SENSOR_A 35      
#define PINO_SENSOR_B 34      
#define PINO_RELE_BOMBA 32    
#define PINO_RELE_VALVULA1 33 
#define PINO_RELE_VALVULA2 25 
#define LED_PIN 2

//////////////////////
// PARÂMETROS DO SENSOR
//////////////////////
const float PRESSAO_MAXIMA_FS = 30.0; 
const float TENSAO_MIN_SENSOR = 0.5;  
const float TENSAO_MAX_SENSOR = 4.5;  
const float TENSAO_MAX_ADC_ESP = 3.3; 
const float FATOR_DIVISOR = 0.681;    

const float TENSAO_SENSOR_MIN_VALIDA = 0.35;
const float TENSAO_SENSOR_MAX_VALIDA = 4.65;

//////////////////////
// VARIÁVEIS DE LÓGICA E SEGURANÇA
//////////////////////
float pressaoRefA = 0;
float pressaoRefB = 0;
float pressaoAtualA = 0;
float pressaoAtualB = 0;

const float MARGEM_TOLERANCIA = 0.80; 
const float PRESSAO_REF_MINIMA = 0.2; 
const float MAX_OSCILACAO_CALIBRACAO = 0.25; 

const uint32_t TEMPO_CONFIRMACAO = 3000; 
const uint32_t MAX_INTERVALO_AMOSTRA = 100; // ms
const float DELTA_MIN_LOCALIZACAO = 0.10;   

// Watchdog do Ciclo Seguro
const uint32_t MAX_INTERVALO_LOOP_SEGURO = 250; 
uint32_t ultimoCicloSeguro = 0;
bool watchdogInicializado = false;

struct DebounceQueda {
  bool ativo = false;
  uint32_t ultimo = 0;
  uint32_t acumulado = 0;
  bool confirmado = false;
};
DebounceQueda dbA, dbB;

struct LeituraPressao {
  float pressao;
  bool valido;
};

// Máquina de Estados (Falha Segura em Duas Etapas)
enum EstadoSistema {
  CALIBRANDO,
  MONITORANDO,
  AGUARDANDO_ALIVIO_BOMBA,
  ISOLADO,
  FALHA_AGUARDANDO_ALIVIO,
  FALHA_SEGURA
};
EstadoSistema estadoAtual = CALIBRANDO;

uint32_t inicioAlivio = 0;
const uint32_t TEMPO_ALIVIO_BOMBA = 300; 
const uint32_t TEMPO_ESTABILIZACAO = 8000;
const uint32_t INTERVALO_CHECK_CALIBRACAO = 50;

int trechoVazamento = 0; 
bool falhaTempoAmostragem = false; 

//////////////////////
// CONTROLE DE REDE 
//////////////////////
const uint32_t INTERVALO_RETRY_MQTT = 5000;
uint32_t ultimaTentativaMQTT = 0;
uint32_t ultimoEnvioMQTT = 0;

//////////////////////
// FUNÇÕES DE LEITURA E VALIDAÇÃO
//////////////////////
LeituraPressao lerPressaoSegura(int pino) {
  int valorBruto = analogRead(pino);
  float tensaoPinoESP = (valorBruto / 4095.0) * TENSAO_MAX_ADC_ESP;
  float tensaoSensor = tensaoPinoESP / FATOR_DIVISOR;

  if (tensaoSensor < TENSAO_SENSOR_MIN_VALIDA || tensaoSensor > TENSAO_SENSOR_MAX_VALIDA) {
    return {0.0, false};
  }

  if (tensaoSensor <= TENSAO_MIN_SENSOR) return {0.0, true};

  float pressao = (tensaoSensor - TENSAO_MIN_SENSOR) * (PRESSAO_MAXIMA_FS / (TENSAO_MAX_SENSOR - TENSAO_MIN_SENSOR));
  if (pressao > PRESSAO_MAXIMA_FS) pressao = PRESSAO_MAXIMA_FS;
  return {pressao, true};
}

void limparDebounces() {
  dbA.ativo = false; dbA.acumulado = 0; dbA.confirmado = false;
  dbB.ativo = false; dbB.acumulado = 0; dbB.confirmado = false;
}

// SEQUÊNCIA DE EMERGÊNCIA (Duas Etapas)
void solicitarFalhaSegura(const char* motivo, uint32_t agora) {
  digitalWrite(PINO_RELE_BOMBA, LOW); // Corta a força motriz PRIMEIRO
  limparDebounces();
  
  inicioAlivio = agora;
  estadoAtual = FALHA_AGUARDANDO_ALIVIO;
  
  Serial.println(">>> FALHA CRITICA DETETADA - BOMBA DESLIGADA <<<");
  Serial.println(motivo);
}

void verificarCadenciaLoop(uint32_t agora) {
  if (estadoAtual != MONITORANDO) {
    ultimoCicloSeguro = agora;
    watchdogInicializado = true;
    return;
  }
  if (watchdogInicializado && (agora - ultimoCicloSeguro > MAX_INTERVALO_LOOP_SEGURO)) {
    solicitarFalhaSegura("Loop perdeu cadencia critica.", agora);
    return;
  }
  ultimoCicloSeguro = agora;
  watchdogInicializado = true;
}

void processarSeguranca(uint32_t agora); 

//////////////////////
// FUNÇÃO DE CALIBRAÇÃO 
//////////////////////
bool aguardarEstabilizacaoMonitorada() {
  uint32_t inicio = millis();
  uint32_t ultimaLeitura = 0;
  
  const uint32_t TEMPO_MAX_SEM_PRESSAO = 3000;
  uint32_t inicioSemPressao = 0;

  while (millis() - inicio < TEMPO_ESTABILIZACAO) {
    uint32_t agora = millis();

    if (agora - ultimaLeitura >= INTERVALO_CHECK_CALIBRACAO) {
      ultimaLeitura = agora;

      LeituraPressao lA = lerPressaoSegura(PINO_SENSOR_A);
      LeituraPressao lB = lerPressaoSegura(PINO_SENSOR_B);

      // Proteção 1: Validação Elétrica
      if (!lA.valido || !lB.valido) {
        solicitarFalhaSegura("Erro eletrico durante estabilizacao inicial.", agora);
        return false;
      }
      
      // Proteção 2: Validação Hidráulica (Bomba desferrada / Furo gigante)
      if (lA.pressao <= PRESSAO_REF_MINIMA || lB.pressao <= PRESSAO_REF_MINIMA) {
        if (inicioSemPressao == 0) inicioSemPressao = agora;

        if (agora - inicioSemPressao >= TEMPO_MAX_SEM_PRESSAO) {
          solicitarFalhaSegura("Sem pressurizacao hidraulica na estabilizacao.", agora);
          return false;
        }
      } else {
        inicioSemPressao = 0;
      }
    }

    processarSeguranca(agora); 
    delay(1);
  }
  return true;
}

void calibrarSistema() {
  estadoAtual = CALIBRANDO;
  Serial.println("\n>>> INICIANDO CALIBRACAO <<<");
  
  digitalWrite(PINO_RELE_VALVULA1, HIGH);
  digitalWrite(PINO_RELE_VALVULA2, HIGH);
  digitalWrite(PINO_RELE_BOMBA, HIGH);
  
  if (!aguardarEstabilizacaoMonitorada()) {
    return; 
  }
  
  float somaA = 0, somaB = 0;
  float minA = 9999, maxA = -9999;
  float minB = 9999, maxB = -9999;

  for(int i = 0; i < 50; i++) {
    LeituraPressao lA = lerPressaoSegura(PINO_SENSOR_A);
    LeituraPressao lB = lerPressaoSegura(PINO_SENSOR_B);
    
    if (!lA.valido || !lB.valido) {
      solicitarFalhaSegura("Erro eletrico no sensor durante media de calibracao.", millis());
      return;
    }
    minA = min(minA, lA.pressao); maxA = max(maxA, lA.pressao);
    minB = min(minB, lB.pressao); maxB = max(maxB, lB.pressao);
    somaA += lA.pressao;
    somaB += lB.pressao;
    delay(50);
  }
  
  if ((maxA - minA) > MAX_OSCILACAO_CALIBRACAO || (maxB - minB) > MAX_OSCILACAO_CALIBRACAO) {
    solicitarFalhaSegura("Pressao instavel durante calibracao.", millis());
    return;
  }

  pressaoRefA = somaA / 50.0;
  pressaoRefB = somaB / 50.0;

  if (pressaoRefA <= PRESSAO_REF_MINIMA || pressaoRefB <= PRESSAO_REF_MINIMA) {
    solicitarFalhaSegura("Pressao base muito baixa pos-estabilizacao.", millis());
    return;
  }
  
  ultimoCicloSeguro = millis(); 
  watchdogInicializado = true;
  estadoAtual = MONITORANDO;
  Serial.println(">>> CALIBRACAO BEM SUCEDIDA. SISTEMA EM MONITORAMENTO! <<<");
}

//////////////////////
// LÓGICA DE SEGURANÇA E DEBOUNCE
//////////////////////
void atualizarDebounce(DebounceQueda &db, bool queda, uint32_t agora) {
  if (!queda) {
    db.ativo = false; db.acumulado = 0; db.confirmado = false;
    return;
  }
  if (!db.ativo) {
    db.ativo = true; db.ultimo = agora; db.acumulado = 0; db.confirmado = false;
    return;
  }

  uint32_t dt = agora - db.ultimo;
  db.ultimo = agora;

  if (dt > MAX_INTERVALO_AMOSTRA) {
    falhaTempoAmostragem = true;
    return;
  }

  db.acumulado += dt;
  if (db.acumulado >= TEMPO_CONFIRMACAO) db.confirmado = true;
}

int escolherTrecho(bool confA, bool confB, bool quedaA, bool quedaB) {
  if (quedaA && quedaB) {
    float quedaRelA = (pressaoRefA - pressaoAtualA) / pressaoRefA;
    float quedaRelB = (pressaoRefB - pressaoAtualB) / pressaoRefB;

    if (fabsf(quedaRelA - quedaRelB) <= DELTA_MIN_LOCALIZACAO) return 0; 
    return (quedaRelA > quedaRelB) ? 1 : 2;
  }

  if (confA) return 1;
  if (confB) return 2;
  return 0;
}

bool suspeitaVazamentoEmAndamento() {
  return estadoAtual == MONITORANDO && (dbA.ativo || dbB.ativo);
}

void processarSeguranca(uint32_t agora) {
  if (estadoAtual == MONITORANDO) {
    bool quedaA = pressaoAtualA < (pressaoRefA * MARGEM_TOLERANCIA);
    bool quedaB = pressaoAtualB < (pressaoRefB * MARGEM_TOLERANCIA);

    falhaTempoAmostragem = false;
    atualizarDebounce(dbA, quedaA, agora);
    atualizarDebounce(dbB, quedaB, agora);

    if (falhaTempoAmostragem && (quedaA || quedaB)) {
      solicitarFalhaSegura("Perda de cadencia na suspeita.", agora);
      return;
    }

    if (dbA.confirmado || dbB.confirmado) {
      trechoVazamento = escolherTrecho(dbA.confirmado, dbB.confirmado, quedaA, quedaB);
      limparDebounces(); 
      
      digitalWrite(PINO_RELE_BOMBA, LOW); // Corta a bomba imediatamente
      inicioAlivio = agora;
      
      if (trechoVazamento == 0) {
        estadoAtual = FALHA_AGUARDANDO_ALIVIO;
      } else {
        estadoAtual = AGUARDANDO_ALIVIO_BOMBA;
      }
      // CORREÇÃO: Serial.println removido daqui para não atrasar a máquina no microssegundo crítico
    }
  } 
  else if (estadoAtual == AGUARDANDO_ALIVIO_BOMBA) {
    if (agora - inicioAlivio >= TEMPO_ALIVIO_BOMBA) {
      if (trechoVazamento == 1) digitalWrite(PINO_RELE_VALVULA1, LOW);
      else if (trechoVazamento == 2) digitalWrite(PINO_RELE_VALVULA2, LOW);
      estadoAtual = ISOLADO;
      
      // CORREÇÃO: Mensagem movida para cá, quando o isolamento mecânico já terminou
      Serial.println("!!!! VAZAMENTO SETORIAL ISOLADO - VALVULA ATUADA !!!!");
    }
  }
  else if (estadoAtual == FALHA_AGUARDANDO_ALIVIO) {
    if (agora - inicioAlivio >= TEMPO_ALIVIO_BOMBA) {
      digitalWrite(PINO_RELE_VALVULA1, LOW);
      digitalWrite(PINO_RELE_VALVULA2, LOW);
      estadoAtual = FALHA_SEGURA;
      
      // CORREÇÃO: Mensagem movida para cá, pós-isolamento total
      Serial.println(">>> SISTEMA TRANCADO EM FALHA SEGURA GLOBAL <<<");
    }
  }
}

//////////////////////
// FUNÇÕES DE REDE 
//////////////////////
void manterMQTT(uint32_t agora) {
  if (WiFi.status() != WL_CONNECTED) return; 
  
  if (!client.connected()) {
    if (agora - ultimaTentativaMQTT >= INTERVALO_RETRY_MQTT) {
      ultimaTentativaMQTT = agora;
      client.connect("ESP32_SACOP"); 
    }
    return;
  }
  client.loop(); 
}

void PUBLICAR_MQTT(uint32_t agora) {
  if (client.connected() && (agora - ultimoEnvioMQTT >= 2000)) {
    ultimoEnvioMQTT = agora;
    
    char json[128];
    snprintf(json, sizeof(json), "{\"pA\":%.2f,\"pB\":%.2f,\"estado\":%d,\"trecho\":%d}", 
             pressaoAtualA, pressaoAtualB, estadoAtual, trechoVazamento);

    client.publish("esp32/sensores", json);
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
  }
}

//////////////////////
// SETUP E LOOP 
//////////////////////
void setup() {
  Serial.begin(115200);
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(PINO_RELE_BOMBA, OUTPUT);
  pinMode(PINO_RELE_VALVULA1, OUTPUT);
  pinMode(PINO_RELE_VALVULA2, OUTPUT);
  
  digitalWrite(PINO_RELE_BOMBA, LOW);
  digitalWrite(PINO_RELE_VALVULA1, LOW);
  digitalWrite(PINO_RELE_VALVULA2, LOW);

  analogReadResolution(12); 
  analogSetPinAttenuation(PINO_SENSOR_A, ADC_11db);
  analogSetPinAttenuation(PINO_SENSOR_B, ADC_11db);

  WiFi.begin(ssid, password);
  client.setServer(mqtt_server, 1883);

  client.setSocketTimeout(1);
  client.setKeepAlive(10);
  
  calibrarSistema();
}

void loop() {
  uint32_t agora = millis();

  // 0. WATCHDOG GLOBAL
  verificarCadenciaLoop(agora);

  // 1. OBRIGAÇÃO LOCAL E LEITURA FÍSICA
  if (estadoAtual != FALHA_SEGURA && estadoAtual != AGUARDANDO_ALIVIO_BOMBA && estadoAtual != FALHA_AGUARDANDO_ALIVIO) {
    LeituraPressao lA = lerPressaoSegura(PINO_SENSOR_A);
    LeituraPressao lB = lerPressaoSegura(PINO_SENSOR_B);
    
    if (!lA.valido || !lB.valido) {
      solicitarFalhaSegura("Sinal Eletrico Perdido/Oscilante nos Sensores.", agora);
    } else {
      pressaoAtualA = lA.pressao;
      pressaoAtualB = lB.pressao;
    }
  }
  
  // 2. SEGURANÇA E INTERTRAVAMENTOS PENDENTES
  processarSeguranca(agora);

  // 3. BARREIRA DE PRIORIDADE MÁXIMA 
  if (estadoAtual == AGUARDANDO_ALIVIO_BOMBA || estadoAtual == FALHA_AGUARDANDO_ALIVIO || suspeitaVazamentoEmAndamento()) {
    return; // Aborta rotinas não críticas e protege os tempos amostrais
  }

  // 4. REDE E TELEMETRIA
  manterMQTT(agora);
  PUBLICAR_MQTT(agora);
}