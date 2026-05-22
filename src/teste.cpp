#include <Arduino.h>
#define PINO_VALVULA1 33 
#define PINO_VALVULA2 25 
#define PINO_BOMBA 32    
#define LED_PIN 2
#define PINO_BOTAO_TESTE 26

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(PINO_BOMBA, OUTPUT);
  pinMode(PINO_VALVULA1, OUTPUT);
  pinMode(PINO_VALVULA2, OUTPUT);

  digitalWrite(PINO_BOMBA, LOW);
  digitalWrite(PINO_VALVULA1, LOW);
  digitalWrite(PINO_VALVULA2, LOW);

  pinMode(PINO_BOTAO_TESTE, INPUT_PULLDOWN);
}

void loop() {

  if (digitalRead(PINO_BOTAO_TESTE) == HIGH) {
    digitalWrite(PINO_BOMBA, HIGH);
    digitalWrite(PINO_VALVULA1, HIGH);
    digitalWrite(PINO_VALVULA2, HIGH);
    digitalWrite(LED_PIN, HIGH); 
  } else {
   
    digitalWrite(PINO_BOMBA, LOW);
    digitalWrite(PINO_VALVULA1, LOW);
    digitalWrite(PINO_VALVULA2, LOW);
    digitalWrite(LED_PIN, LOW);
  }
}