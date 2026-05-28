#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>
#include <ctype.h>

//////////////////////
// CONFIG WIFI E MQTT
//////////////////////
const char* ssid = "Joao Arthur 5G";
const char* password = "190421pl";
const char* mqtt_server = "192.168.0.102";

WiFiClient espClient;
PubSubClient client(espClient);
bool avisoConexaoPerdida = false;

//////////////////////
// PINOS DE HARDWARE
//////////////////////
#define PINO_SENSOR_A 35
#define PINO_SENSOR_B 34
#define PINO_RELE_BOMBA 32
#define PINO_RELE_VALVULA1 33
#define PINO_RELE_VALVULA2 25
#define PINO_BOTAO_START 27
#define LED_PIN 2

const char* TOPICO_COMANDO = "esp32/comando";
const char* CLIENT_ID_MQTT = "ESP32_SACOP";

//////////////////////
// PARÂMETROS DO SENSOR E HARDWARE
//////////////////////
const float PRESSAO_MAXIMA_FS = 30.0;
const float TENSAO_MIN_SENSOR_A = 0.49; // Zero físico exato (S1)
const float TENSAO_MIN_SENSOR_B = 0.25; // Zero físico exato (S2)
const float TENSAO_MAX_SENSOR = 4.5;
const float TENSAO_MAX_ADC_ESP = 3.3;

const float FATOR_DIVISOR = 10.0 / (4.7 + 10.0);

const float TENSAO_SENSOR_MIN_VALIDA = 0.10;
const float TENSAO_SENSOR_MAX_VALIDA = 4.65;

//////////////////////
// VARIÁVEIS DE LÓGICA E SEGURANÇA
//////////////////////
float pressaoRefA = 0;
float pressaoRefB = 0;
float pressaoAtualA = 0;
float pressaoAtualB = 0;

// Limites de Segurança com Histerese (Zona Morta)
const float MARGEM_TOLERANCIA = 0.85; // Caiu 15% -> Inicia o cronômetro
const float MARGEM_RESET      = 0.92; // Subiu para 92% -> Aborta o alarme

bool estadoQuedaA = false;
bool estadoQuedaB = false;

const float PRESSAO_REF_MINIMA = 0.05;
const float MAX_OSCILACAO_CALIBRACAO = 7.00; // Margem para pulsação da bomba

const uint32_t TEMPO_CONFIRMACAO = 6000; // 6 segundos ininterruptos para confirmar o vazamento
const uint32_t MAX_INTERVALO_AMOSTRA = 100;
const float DELTA_MIN_LOCALIZACAO = 0.10;

const uint32_t MAX_INTERVALO_LOOP_SEGURO = 1000;
uint32_t ultimoCicloSeguro = 0;

const uint32_t TEMPO_ALIVIO_BOMBA = 300;
const uint32_t TEMPO_ESTABILIZACAO = 12000; 
const uint32_t INTERVALO_CHECK_CALIBRACAO = 50;
const uint32_t TEMPO_MAX_SEM_PRESSAO = 8000; 
const uint32_t TEMPO_DEBOUNCE_BOTAO = 50;

const uint32_t INTERVALO_RETRY_MQTT = 5000;
const uint32_t TEMPO_CONEXAO_WIFI_INICIAL = 8000;
uint32_t ultimaTentativaMQTT = 0;
uint32_t ultimoEnvioMQTT = 0;

bool pedidoStart = false;

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
  AGUARDANDO_START,
  CALIBRANDO,
  MONITORANDO,
  AGUARDANDO_ALIVIO_BOMBA,
  ISOLADO,
  FALHA_AGUARDANDO_ALIVIO,
  FALHA_SEGURA
};

DebounceQueda dbA, dbB;
EstadoSistema estadoAtual = AGUARDANDO_START;

uint32_t inicioAlivio = 0;
int trechoVazamento = 0;

//////////////////////
// FUNÇÃO MESTRE DE LEITURA (ALTA VELOCIDADE)
//////////////////////
LeituraPressao lerPressaoSegura(int pino) {
  uint32_t milivoltsReais = analogReadMilliVolts(pino);
  float tensaoPino = milivoltsReais / 1000.0;
  float tensaoSensor = tensaoPino / FATOR_DIVISOR;

  if (tensaoSensor < TENSAO_SENSOR_MIN_VALIDA || tensaoSensor > TENSAO_SENSOR_MAX_VALIDA) {
    return {0.0, false};
  }

  float tensaoMinimaReal = (pino == PINO_SENSOR_A) ? TENSAO_MIN_SENSOR_A : TENSAO_MIN_SENSOR_B;
  if (tensaoSensor <= tensaoMinimaReal) return {0.0, true};

  float fatorConversaoPressao = PRESSAO_MAXIMA_FS / (TENSAO_MAX_SENSOR - tensaoMinimaReal);
  float pressao = (tensaoSensor - tensaoMinimaReal) * fatorConversaoPressao;
  
  if (pressao > PRESSAO_MAXIMA_FS) pressao = PRESSAO_MAXIMA_FS;

  return {pressao, true}; 
}

void limparDebounces() {
  dbA = DebounceQueda{};
  dbB = DebounceQueda{};
  estadoQuedaA = false;
  estadoQuedaB = false;
}

void colocarSaidasEmRepouso() {
  digitalWrite(PINO_RELE_BOMBA, LOW);
  digitalWrite(PINO_RELE_VALVULA1, LOW);
  digitalWrite(PINO_RELE_VALVULA2, LOW);
}

void solicitarFalhaSegura(const char* motivo, uint32_t agora) {
  digitalWrite(PINO_RELE_BOMBA, LOW);
  limparDebounces();

  inicioAlivio = agora;
  estadoAtual = FALHA_AGUARDANDO_ALIVIO;

  Serial.println("\n>>> FALHA CRITICA DETECTADA - BOMBA DESLIGADA <<<");
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
  Serial.println("\n>>> INICIANDO CALIBRACAO DINAMICA <<<");

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
      solicitarFalhaSegura("Erro eletrico no sensor durante captura de media.", millis());
      return;
    }

    minA = min(minA, lA.pressao);
    maxA = max(maxA, lA.pressao);
    minB = min(minB, lB.pressao);
    maxB = max(maxB, lB.pressao);

    somaA += lA.pressao;
    somaB += lB.pressao;

    delay(20); 
  }

  float deltaA = maxA - minA;
  float deltaB = maxB - minB;

  if (deltaA > MAX_OSCILACAO_CALIBRACAO || deltaB > MAX_OSCILACAO_CALIBRACAO) {
    Serial.println("\n!!! DIAGNOSTICO DE TURBULENCIA EXTREMA !!!");
    Serial.print("Variacao S1: "); Serial.print(deltaA); Serial.println(" PSI");
    Serial.print("Variacao S2: "); Serial.print(deltaB); Serial.println(" PSI");
    solicitarFalhaSegura("Pressao oscilou acima do limite de operacao segura.", millis());
    return;
  }

  pressaoRefA = pressaoAtualA = somaA / 50.0;
  pressaoRefB = pressaoAtualB = somaB / 50.0;

  if (pressaoRefA <= PRESSAO_REF_MINIMA || pressaoRefB <= PRESSAO_REF_MINIMA) {
    solicitarFalhaSegura("Pressao base estagnou em nivel muito baixo.", millis());
    return;
  }

  ultimoCicloSeguro = millis();
  estadoAtual = MONITORANDO;

  Serial.println(">>> CALIBRACAO BEM SUCEDIDA. SISTEMA EM MONITORAMENTO! <<<");
  Serial.print("Referencia ajustada para operacao: "); Serial.print(pressaoRefA); Serial.println(" PSI");
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
    
    // --- LÓGICA DE HISTERESE (ZONA MORTA) ---
    float limiteAlarmeA = pressaoRefA * MARGEM_TOLERANCIA;
    float limiteResetA  = pressaoRefA * MARGEM_RESET;
    float limiteAlarmeB = pressaoRefB * MARGEM_TOLERANCIA;
    float limiteResetB  = pressaoRefB * MARGEM_RESET;

    if (pressaoAtualA <= limiteAlarmeA) {
      estadoQuedaA = true;
    } else if (pressaoAtualA >= limiteResetA) {
      estadoQuedaA = false;
    }

    if (pressaoAtualB <= limiteAlarmeB) {
      estadoQuedaB = true;
    } else if (pressaoAtualB >= limiteResetB) {
      estadoQuedaB = false;
    }

    bool falhaAmostragem =
      atualizarDebounce(dbA, estadoQuedaA, agora) ||
      atualizarDebounce(dbB, estadoQuedaB, agora);

    if (falhaAmostragem) {
      solicitarFalhaSegura("Perda de cadencia na analise de suspeita.", agora);
      return;
    }

    // GATILHO DE CONFIRMAÇÃO DE VAZAMENTO
    if (dbA.confirmado || dbB.confirmado) {
      trechoVazamento = escolherTrecho(dbA.confirmado, dbB.confirmado, estadoQuedaA, estadoQuedaB);
      limparDebounces();

      digitalWrite(PINO_RELE_BOMBA, LOW);
      inicioAlivio = agora;

      Serial.println("\n==================================================");
      Serial.println("!!! ALARME: QUEDA DE PRESSAO CONFIRMADA !!!");
      
      if (trechoVazamento == 1) {
        Serial.println("-> Localizacao estimada: TRECHO A (Antes do Sensor 1)");
      } else if (trechoVazamento == 2) {
        Serial.println("-> Localizacao estimada: TRECHO B (Entre sensores S1 e S2)");
      } else {
        Serial.println("-> Localizacao estimada: TRECHO C (Apos o Sensor 2)");
      }
      Serial.println("==================================================");

      estadoAtual = (trechoVazamento == 0) ? FALHA_AGUARDANDO_ALIVIO : AGUARDANDO_ALIVIO_BOMBA;
    }
    
  } else if (estadoAtual == AGUARDANDO_ALIVIO_BOMBA) {
    if (agora - inicioAlivio >= TEMPO_ALIVIO_BOMBA) {
      if (trechoVazamento == 1) digitalWrite(PINO_RELE_VALVULA1, LOW);
      else if (trechoVazamento == 2) digitalWrite(PINO_RELE_VALVULA2, LOW);

      estadoAtual = ISOLADO;
      Serial.println("\n>>> VAZAMENTO SETORIAL ISOLADO - VALVULA ATUADA <<<");
    }
  } else if (estadoAtual == FALHA_AGUARDANDO_ALIVIO) {
    if (agora - inicioAlivio >= TEMPO_ALIVIO_BOMBA) {
      digitalWrite(PINO_RELE_VALVULA1, LOW);
      digitalWrite(PINO_RELE_VALVULA2, LOW);

      estadoAtual = FALHA_SEGURA;
      Serial.println("\n>>> SISTEMA TRANCADO EM FALHA SEGURA GLOBAL <<<");
    }
  }
}

bool botaoStartPressionado(uint32_t agora) {
  static bool ultimoLido = HIGH;
  static bool estadoEstavel = HIGH;
  static uint32_t ultimaMudanca = 0;

  bool lido = digitalRead(PINO_BOTAO_START);

  if (lido != ultimoLido) {
    ultimoLido = lido;
    ultimaMudanca = agora;
  }

  if ((agora - ultimaMudanca >= TEMPO_DEBOUNCE_BOTAO) && (lido != estadoEstavel)) {
    estadoEstavel = lido;
    return estadoEstavel == LOW;
  }

  return false;
}

void resetarSistema(const char* origem) {
  Serial.print("\n[RESET] Comando de destravamento recebido por: ");
  Serial.println(origem);

  colocarSaidasEmRepouso();
  limparDebounces();
  pedidoStart = false;
  trechoVazamento = 0;
  pressaoRefA = 0;
  pressaoRefB = 0;
  pressaoAtualA = 0;
  pressaoAtualB = 0;

  estadoAtual = AGUARDANDO_START;
  Serial.println(">>> SISTEMA RESETADO. AGUARDANDO NOVO START <<<");
}

void solicitarStart(const char* origem) {
  if (estadoAtual != AGUARDANDO_START) {
    Serial.print("[START IGNORADO] Maquina ja esta em operacao ou falha. Origem: ");
    Serial.println(origem);
    return;
  }

  pedidoStart = true;
  Serial.print("\n[START] Pedido de partida validado. Origem: ");
  Serial.println(origem);
}

void iniciarSistema() {
  pedidoStart = false;
  trechoVazamento = 0;
  pressaoRefA = 0;
  pressaoRefB = 0;
  pressaoAtualA = 0;
  pressaoAtualB = 0;
  limparDebounces();
  colocarSaidasEmRepouso();

  calibrarSistema();
}

void tratarStart(uint32_t agora) {
  if (botaoStartPressionado(agora)) {
    // Se a máquina estiver travada, o botão atua como RESET
    if (estadoAtual == FALHA_SEGURA || estadoAtual == ISOLADO || estadoAtual == FALHA_AGUARDANDO_ALIVIO) {
      resetarSistema("Painel Local (Botao Fisico)");
    } else {
      // Caso contrário, atua como START
      solicitarStart("Painel Local (Botao Fisico)");
    }
  }

  if (pedidoStart && estadoAtual == AGUARDANDO_START) {
    iniciarSistema();
  } else if (pedidoStart) {
    pedidoStart = false;
  }
}

void callbackMQTT(char* topic, byte* payload, unsigned int length) {
  char msg[16];
  unsigned int len = min(length, (unsigned int)(sizeof(msg) - 1));

  memcpy(msg, payload, len);
  msg[len] = '\0';

  while (len > 0 && isspace((unsigned char)msg[len - 1])) {
    msg[--len] = '\0';
  }

  for (unsigned int i = 0; i < len; i++) {
    msg[i] = toupper((unsigned char)msg[i]);
  }

  // Verifica se o tópico corresponde ao tópico de comando esperado
  if (strcmp(topic, TOPICO_COMANDO) == 0) {
    if (strcmp(msg, "START") == 0) {
      solicitarStart("Painel Remoto (Node-RED)");
    } else if (strcmp(msg, "RESET") == 0) {
      // Verifica se o sistema realmente precisa de reset antes de atuar
      if (estadoAtual == FALHA_SEGURA || estadoAtual == ISOLADO || estadoAtual == FALHA_AGUARDANDO_ALIVIO) {
        resetarSistema("Painel Remoto (Node-RED)");
      } else {
        Serial.println("\n[RESET IGNORADO] O sistema nao esta em estado de falha.");
      }
    }
  }
}

bool conectarMQTT() {
  if (!client.connect(CLIENT_ID_MQTT)) return false;
  client.subscribe(TOPICO_COMANDO);
  return true;
}

void conectarRedeInicial() {
  Serial.println(">>> INICIANDO SUBSISTEMA DE REDE <<<");
  uint32_t inicio = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - inicio < TEMPO_CONEXAO_WIFI_INICIAL) {
    delay(100);
  }

  if (WiFi.status() != WL_CONNECTED) {
    avisoConexaoPerdida = true;
    Serial.println("[AVISO] WiFi indisponivel no arranque. Operacao restrita ao controle local.");
    return;
  }

  Serial.print("[INFO] WiFi conectado. IP: ");
  Serial.println(WiFi.localIP());

  if (conectarMQTT()) {
    avisoConexaoPerdida = false;
    Serial.println("[INFO] Servidor MQTT estabelecido. Node-RED pronto.");
  } else {
    avisoConexaoPerdida = true;
    Serial.println("[AVISO] Falha de conexao com MQTT. Node-RED indisponivel.");
  }
}

void manterMQTT(uint32_t agora) {
  if (WiFi.status() != WL_CONNECTED) {
    if (!avisoConexaoPerdida) {
      Serial.println("\n[AVISO] Sinal WiFi perdido! Sistema segue mantendo a seguranca via hardware.");
      avisoConexaoPerdida = true;
    }
    return;
  }

  if (!client.connected()) {
    if (!avisoConexaoPerdida) {
      Serial.println("\n[AVISO] Conexao com Broker MQTT derrubada.");
      avisoConexaoPerdida = true;
    }

    if (estadoAtual == MONITORANDO || estadoAtual == CALIBRANDO) {
      return; 
    }

    if (agora - ultimaTentativaMQTT >= INTERVALO_RETRY_MQTT) {
      ultimaTentativaMQTT = agora;
      if (conectarMQTT()) {
        avisoConexaoPerdida = false;
        Serial.println("\n[INFO] Rede reestabelecida com sucesso.");
      }
    }
    return;
  }

  if (avisoConexaoPerdida) {
    Serial.println("\n[INFO] Todas as conexoes de rede operacionais.");
    avisoConexaoPerdida = false;
  }
  client.loop();
}

void publicarMQTT(uint32_t agora) {
  if (client.connected() && (agora - ultimoEnvioMQTT >= 2000)) {
    ultimoEnvioMQTT = agora;

    char json[160];
    snprintf(json, sizeof(json),
             "{\"pA\":%.2f,\"pB\":%.2f,\"estado\":%d,\"trecho\":%d,\"start\":%d}",
             pressaoAtualA, pressaoAtualB, estadoAtual, trechoVazamento,
             estadoAtual != AGUARDANDO_START);

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
  pinMode(PINO_BOTAO_START, INPUT_PULLUP);

  colocarSaidasEmRepouso();

  analogReadResolution(12);
  analogSetPinAttenuation(PINO_SENSOR_A, ADC_11db);
  analogSetPinAttenuation(PINO_SENSOR_B, ADC_11db);

  WiFi.begin(ssid, password);
  client.setServer(mqtt_server, 1883);
  client.setCallback(callbackMQTT);
  client.setSocketTimeout(1);
  client.setKeepAlive(10);

  conectarRedeInicial();

  estadoAtual = AGUARDANDO_START;
  Serial.println("\n>>> SISTEMA SEGURO. AGUARDANDO COMANDO DE START <<<");
}

void loop() {
  uint32_t agora = millis();

  verificarCadenciaLoop(agora);
  tratarStart(agora);

  if (estadoAtual == MONITORANDO) {
    LeituraPressao lA = lerPressaoSegura(PINO_SENSOR_A);
    LeituraPressao lB = lerPressaoSegura(PINO_SENSOR_B);
    
    if (!lA.valido || !lB.valido) {
      solicitarFalhaSegura("Sinal eletrico corrompido nos sensores (Falha de HW).", agora);
    } else {
      // --- FILTRO PASSA-BAIXA (AMORTECEDOR DIGITAL) ---
      static uint32_t ultimaAtualizacaoFiltro = 0;
      
      // Executa a matemática a 100Hz (a cada 10ms), garantindo fluidez sem travar
      if (agora - ultimaAtualizacaoFiltro >= 10) { 
        pressaoAtualA = (lA.pressao * 0.10) + (pressaoAtualA * 0.90);
        pressaoAtualB = (lB.pressao * 0.10) + (pressaoAtualB * 0.90);
        ultimaAtualizacaoFiltro = agora;
      }
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