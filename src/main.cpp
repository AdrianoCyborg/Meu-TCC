#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>

const char* ssid = "Joao Arthur 5G";
const char* password = "190421pl";
const char* mqtt_server = "192.168.0.108";

WiFiClient espClient;
PubSubClient client(espClient);
bool avisoConexaoPerdida = false;

#define PINO_SENSOR_A 35
#define PINO_SENSOR_B 34
#define PINO_RELE_BOMBA 32
#define PINO_RELE_VALVULA1 33
#define PINO_RELE_VALVULA2 25
#define LED_PIN 2

const float PRESSAO_MAXIMA_FS = 30.0;
const float TENSAO_MIN_SENSOR = 0.41;
const float TENSAO_MAX_SENSOR = 4.5;
const float TENSAO_MAX_ADC_ESP = 3.3;

const float FATOR_DIVISOR = 10.0 / (4.7 + 10.0);
const float ADC_PARA_TENSAO_SENSOR = TENSAO_MAX_ADC_ESP / 4095.0 / FATOR_DIVISOR;
const float SENSOR_PARA_PRESSAO = PRESSAO_MAXIMA_FS / (TENSAO_MAX_SENSOR - TENSAO_MIN_SENSOR);

const float TENSAO_SENSOR_MIN_VALIDA = 0.20;
const float TENSAO_SENSOR_MAX_VALIDA = 4.65;

float pressaoRefA = 0;
float pressaoRefB = 0;
float pressaoAtualA = 0;
float pressaoAtualB = 0;

const float MARGEM_TOLERANCIA = 0.90;
const float PRESSAO_REF_MINIMA = 0.05;
const float MAX_OSCILACAO_CALIBRACAO = 0.10;

const uint32_t TEMPO_CONFIRMACAO = 3000;
const uint32_t MAX_INTERVALO_AMOSTRA = 100;
const float DELTA_MIN_LOCALIZACAO = 0.10;

const uint32_t MAX_INTERVALO_LOOP_SEGURO = 250;
uint32_t ultimoCicloSeguro = 0;

const uint32_t TEMPO_ALIVIO_BOMBA = 300;
const uint32_t TEMPO_ESTABILIZACAO = 8000;
const uint32_t INTERVALO_CHECK_CALIBRACAO = 50;
const uint32_t TEMPO_MAX_SEM_PRESSAO = 3000;

const uint32_t INTERVALO_RETRY_MQTT = 5000;
const uint32_t TEMPO_CONEXAO_WIFI_INICIAL = 8000;
uint32_t ultimaTentativaMQTT = 0;
uint32_t ultimoEnvioMQTT = 0;

struct DebounceQueda {
  bool ativo = false;
  uint32_t ultimo = 0;
  uint32_t acumulado = 0;
  bool confirmado = false;
};

struct LeituraPressao {
  float pressao;
  bool valido;
};

enum EstadoSistema {
  CALIBRANDO,
  MONITORANDO,
  AGUARDANDO_ALIVIO_BOMBA,
  ISOLADO,
  FALHA_AGUARDANDO_ALIVIO,
  FALHA_SEGURA
};

DebounceQueda dbA, dbB;
EstadoSistema estadoAtual = CALIBRANDO;

uint32_t inicioAlivio = 0;
int trechoVazamento = 0;

LeituraPressao lerPressaoSegura(int pino) {
  int valorBruto = analogRead(pino);
  float tensaoSensor = valorBruto * ADC_PARA_TENSAO_SENSOR;

  if (tensaoSensor < TENSAO_SENSOR_MIN_VALIDA || tensaoSensor > TENSAO_SENSOR_MAX_VALIDA) {
    return {0.0, false};
  }

  if (tensaoSensor <= TENSAO_MIN_SENSOR) return {0.0, true};

  float pressao = (tensaoSensor - TENSAO_MIN_SENSOR) * SENSOR_PARA_PRESSAO;
  if (pressao > PRESSAO_MAXIMA_FS) pressao = PRESSAO_MAXIMA_FS;

  return {pressao, true};
}

void limparDebounces() {
  dbA = DebounceQueda{};
  dbB = DebounceQueda{};
}

void solicitarFalhaSegura(const char* motivo, uint32_t agora) {
  digitalWrite(PINO_RELE_BOMBA, LOW);
  limparDebounces();

  inicioAlivio = agora;
  estadoAtual = FALHA_AGUARDANDO_ALIVIO;

  Serial.println(">>> FALHA CRITICA DETETADA - BOMBA DESLIGADA <<<");
  Serial.println(motivo);
}

void verificarCadenciaLoop(uint32_t agora) {
  if (estadoAtual != MONITORANDO) {
    ultimoCicloSeguro = agora;
    return;
  }

  if (agora - ultimoCicloSeguro > MAX_INTERVALO_LOOP_SEGURO) {
    solicitarFalhaSegura("Loop perdeu cadencia critica.", agora);
    return;
  }

  ultimoCicloSeguro = agora;
}

bool aguardarEstabilizacaoMonitorada() {
  uint32_t inicio = millis();
  uint32_t ultimaLeitura = 0;
  uint32_t inicioSemPressao = 0;

  while (millis() - inicio < TEMPO_ESTABILIZACAO) {
    uint32_t agora = millis();

    if (agora - ultimaLeitura >= INTERVALO_CHECK_CALIBRACAO) {
      ultimaLeitura = agora;

      LeituraPressao lA = lerPressaoSegura(PINO_SENSOR_A);
      LeituraPressao lB = lerPressaoSegura(PINO_SENSOR_B);

      if (!lA.valido || !lB.valido) {
        solicitarFalhaSegura("Erro eletrico durante estabilizacao inicial.", agora);
        return false;
      }

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

  if (!aguardarEstabilizacaoMonitorada()) return;

  float somaA = 0, somaB = 0;
  float minA = 9999, maxA = -9999;
  float minB = 9999, maxB = -9999;

  for (int i = 0; i < 50; i++) {
    LeituraPressao lA = lerPressaoSegura(PINO_SENSOR_A);
    LeituraPressao lB = lerPressaoSegura(PINO_SENSOR_B);

    if (!lA.valido || !lB.valido) {
      solicitarFalhaSegura("Erro eletrico no sensor durante media de calibracao.", millis());
      return;
    }

    minA = min(minA, lA.pressao);
    maxA = max(maxA, lA.pressao);
    minB = min(minB, lB.pressao);
    maxB = max(maxB, lB.pressao);

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
  estadoAtual = MONITORANDO;

  Serial.println(">>> CALIBRACAO BEM SUCEDIDA. SISTEMA EM MONITORAMENTO! <<<");
}

bool atualizarDebounce(DebounceQueda &db, bool queda, uint32_t agora) {
  if (!queda) {
    db = DebounceQueda{};
    return false;
  }

  if (!db.ativo) {
    db.ativo = true;
    db.ultimo = agora;
    return false;
  }

  uint32_t dt = agora - db.ultimo;
  db.ultimo = agora;

  if (dt > MAX_INTERVALO_AMOSTRA) {
    return true;
  }

  db.acumulado += dt;
  db.confirmado = db.acumulado >= TEMPO_CONFIRMACAO;

  return false;
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

    bool falhaAmostragem =
      atualizarDebounce(dbA, quedaA, agora) ||
      atualizarDebounce(dbB, quedaB, agora);

    if (falhaAmostragem) {
      solicitarFalhaSegura("Perda de cadencia na suspeita.", agora);
      return;
    }

    if (dbA.confirmado || dbB.confirmado) {
      trechoVazamento = escolherTrecho(dbA.confirmado, dbB.confirmado, quedaA, quedaB);
      limparDebounces();

      digitalWrite(PINO_RELE_BOMBA, LOW);
      inicioAlivio = agora;

      estadoAtual = (trechoVazamento == 0) ? FALHA_AGUARDANDO_ALIVIO : AGUARDANDO_ALIVIO_BOMBA;
    }
  } else if (estadoAtual == AGUARDANDO_ALIVIO_BOMBA) {
    if (agora - inicioAlivio >= TEMPO_ALIVIO_BOMBA) {
      if (trechoVazamento == 1) digitalWrite(PINO_RELE_VALVULA1, LOW);
      else if (trechoVazamento == 2) digitalWrite(PINO_RELE_VALVULA2, LOW);

      estadoAtual = ISOLADO;
      Serial.println("!!!! VAZAMENTO SETORIAL ISOLADO - VALVULA ATUADA !!!!");
    }
  } else if (estadoAtual == FALHA_AGUARDANDO_ALIVIO) {
    if (agora - inicioAlivio >= TEMPO_ALIVIO_BOMBA) {
      digitalWrite(PINO_RELE_VALVULA1, LOW);
      digitalWrite(PINO_RELE_VALVULA2, LOW);

      estadoAtual = FALHA_SEGURA;
      Serial.println(">>> SISTEMA TRANCADO EM FALHA SEGURA GLOBAL <<<");
    }
  }
}

void conectarRedeInicial() {
  Serial.println(">>> INICIANDO REDE ANTES DA HIDRAULICA <<<");

  uint32_t inicio = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - inicio < TEMPO_CONEXAO_WIFI_INICIAL) {
    delay(100);
  }

  if (WiFi.status() != WL_CONNECTED) {
    avisoConexaoPerdida = true;
    Serial.println("[AVISO] WiFi indisponivel no arranque. Controle hidraulico seguira localmente.");
    return;
  }

  Serial.print("[INFO] WiFi conectado. IP: ");
  Serial.println(WiFi.localIP());

  if (client.connect("ESP32_SACOP")) {
    avisoConexaoPerdida = false;
    Serial.println("[INFO] MQTT conectado antes da calibracao.");
  } else {
    avisoConexaoPerdida = true;
    Serial.println("[AVISO] MQTT indisponivel no arranque. Controle hidraulico seguira localmente.");
  }
}

void manterMQTT(uint32_t agora) {
  if (WiFi.status() != WL_CONNECTED) {
    if (!avisoConexaoPerdida) {
      Serial.println("[AVISO] WiFi perdido. Controle hidraulico continua localmente.");
      avisoConexaoPerdida = true;
    }
    return;
  }

  if (!client.connected()) {
    if (!avisoConexaoPerdida) {
      Serial.println("[AVISO] Broker MQTT perdido. Controle hidraulico continua localmente.");
      avisoConexaoPerdida = true;
    }

    if (estadoAtual == MONITORANDO || estadoAtual == CALIBRANDO) {
      return;
    }

    if (agora - ultimaTentativaMQTT >= INTERVALO_RETRY_MQTT) {
      ultimaTentativaMQTT = agora;
      client.connect("ESP32_SACOP");
    }

    return;
  }

  if (avisoConexaoPerdida) {
    Serial.println("[INFO] Conexao de rede reestabelecida.");
    avisoConexaoPerdida = false;
  }

  client.loop();
}

void publicarMQTT(uint32_t agora) {
  if (client.connected() && (agora - ultimoEnvioMQTT >= 2000)) {
    ultimoEnvioMQTT = agora;

    char json[128];
    snprintf(json, sizeof(json),
             "{\"pA\":%.2f,\"pB\":%.2f,\"estado\":%d,\"trecho\":%d}",
             pressaoAtualA, pressaoAtualB, estadoAtual, trechoVazamento);

    client.publish("esp32/sensores", json);
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}

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

  conectarRedeInicial();
  calibrarSistema();
}

void loop() {
  uint32_t agora = millis();

  verificarCadenciaLoop(agora);

  if (estadoAtual != FALHA_SEGURA &&
      estadoAtual != AGUARDANDO_ALIVIO_BOMBA &&
      estadoAtual != FALHA_AGUARDANDO_ALIVIO) {
    LeituraPressao lA = lerPressaoSegura(PINO_SENSOR_A);
    LeituraPressao lB = lerPressaoSegura(PINO_SENSOR_B);

    if (!lA.valido || !lB.valido) {
      solicitarFalhaSegura("Sinal eletrico perdido/oscilante nos sensores.", agora);
    } else {
      pressaoAtualA = lA.pressao;
      pressaoAtualB = lB.pressao;
    }
  }

  processarSeguranca(agora);

  if (estadoAtual == AGUARDANDO_ALIVIO_BOMBA ||
      estadoAtual == FALHA_AGUARDANDO_ALIVIO ||
      suspeitaVazamentoEmAndamento()) {
    return;
  }

  manterMQTT(agora);
  publicarMQTT(agora);
}