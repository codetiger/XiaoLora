#include "lora.h"
#include <SPI.h>

#define LED_GPIO PIN_LED
Lora *lora;

void setup() {
  pinMode(LED_GPIO, OUTPUT);
  Serial.begin(9600);
  delay(2000);
  Serial.println("Setting up Lora Chip");
  SPI.begin();
  Lora *lora = new Lora();
  Serial.println("Done");
}

void loop() {
  digitalWrite(LED_GPIO, LOW);
  lora->ProcessIrq();
  delay(200);
  digitalWrite(LED_GPIO, HIGH);
  delay(2000);
}