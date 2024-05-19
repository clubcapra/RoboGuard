#ifndef MYLORA_H
#define MYLORA_H
#include <LoRa.h>
#include <Arduino.h>
#include <Timer.h>

void LoRa_init();
void onReceive1(int packetSize);
int lora_update();

extern int lora_state;

#endif