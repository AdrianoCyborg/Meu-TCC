#include <Arduino.h>

// Pinos dos Sensores
#define PINO_SENSOR_A 35      
#define PINO_SENSOR_B 34      

// Pinos dos Atuadores (MOSFETs)
#define PINO_VALVULA1 33 
#define PINO_VALVULA2 25 
#define PINO_BOMBA 32    
#define LED_PIN 2

// Pino do Botão de Teste (Pino livre escolhido)
#define PINO_BOTAO_TESTE 26

// Parâmetros do seu sensor de 30 PSI
const float PRESSAO_MAXIMA_FS = 30.0; 
const float TENSAO_MIN_SENSOR = 0.5;  
const float TENSAO_MAX_SENSOR = 4.5;  
const float TENSAO_MAX_ADC_ESP = 3.3; 
const float FATOR_DIVISOR = 0.681;    

// Função para converter a leitura bruta em PSI
float calcularPressao(int pino) {
  int valorBruto = analogRead(pino);
  float tensaoPinoESP = (valorBruto / 4095.0) * TENSAO_MAX_ADC_ESP;
  float tensaoSensor = tensaoPinoESP / FATOR_DIVISOR;

  // Se estiver abaixo do mínimo elétrico, considera 0 PSI
  if (tensaoSensor <= TENSAO_MIN_SENSOR) return 0.0;

  // Regra de três da curva do sensor
  float pressao = (tensaoSensor - TENSAO_MIN_SENSOR) * (PRESSAO_MAXIMA_FS / (TENSAO_MAX_SENSOR - TENSAO_MIN_SENSOR));
  if (pressao > PRESSAO_MAXIMA_FS) pressao = PRESSAO_MAXIMA_FS;
  
  return pressao;
}

void setup() {
  Serial.begin(115200);
  
  // Configuração dos pinos dos MOSFETs como saída
  pinMode(LED_PIN, OUTPUT);
  pinMode(PINO_BOMBA, OUTPUT);
  pinMode(PINO_VALVULA1, OUTPUT);
  pinMode(PINO_VALVULA2, OUTPUT);
  
  // Garante que tudo comece desligado (0)
  digitalWrite(PINO_BOMBA, LOW);
  digitalWrite(PINO_VALVULA1, LOW);
  digitalWrite(PINO_VALVULA2, LOW);

  // Configuração do botão com resistor de Pull-Down interno
  // O pino fica em 0V por padrão. Quando você conecta o pino 26 ao 3.3V, ele lê HIGH.
  pinMode(PINO_BOTAO_TESTE, INPUT_PULLDOWN);

  // Configuração do ADC do ESP32
  analogReadResolution(12); 
  analogSetPinAttenuation(PINO_SENSOR_A, ADC_11db);
  analogSetPinAttenuation(PINO_SENSOR_B, ADC_11db);

  Serial.println("--- MODO DE TESTE DE HARDWARE INICIADO ---");
  Serial.println("Pressione o botao (Conecte pino 26 ao 3.3V) para acionar os MOSFETs.");
}

void loop() {
  // 1. LEITURA DOS SENSORES
  float pressaoA = calcularPressao(PINO_SENSOR_A);
  float pressaoB = calcularPressao(PINO_SENSOR_B);

  // Mostra os valores no terminal a cada 500ms
  static uint32_t tempoUltimoPrint = 0;
  if (millis() - tempoUltimoPrint >= 500) {
    tempoUltimoPrint = millis();
    Serial.print("Sensor A: ");
    Serial.print(pressaoA);
    Serial.print(" PSI  |  Sensor B: ");
    Serial.print(pressaoB);
    Serial.println(" PSI");
  }

  // 2. VERIFICAÇÃO DO BOTÃO (ACIONAMENTO DAS VÁLVULAS E BOMBA)
  if (digitalRead(PINO_BOTAO_TESTE) == HIGH) {
    // Se recebeu o pulso/sinal de 3.3V, liga tudo (1)
    digitalWrite(PINO_BOMBA, HIGH);
    digitalWrite(PINO_VALVULA1, HIGH);
    digitalWrite(PINO_VALVULA2, HIGH);
    digitalWrite(LED_PIN, HIGH); 
  } else {
    // Se soltou o botão, desliga tudo (0)
    digitalWrite(PINO_BOMBA, LOW);
    digitalWrite(PINO_VALVULA1, LOW);
    digitalWrite(PINO_VALVULA2, LOW);
    digitalWrite(LED_PIN, LOW);
  }

  delay(10); // Pequena pausa para estabilidade do loop
}