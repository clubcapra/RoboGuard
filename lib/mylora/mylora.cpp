#include "mylora.h"

#define TIMEOUT 500


int rcv_flg = 0;
int lora_state = 0;

void LoRa_init()
{
    SPI.setMISO(PB4);
    SPI.setMOSI(PB5);
    SPI.setSCLK(PB3);
    LoRa.setPins(PA15, PC7, PC8);
    LoRa.setTxPower(20);
    
    LoRa.setSyncWord(0xBE);
    if (!LoRa.begin(904600E3)) // frequency
    {
        Serial1.println("\n[ERROR] LoRa init failed");
        abort();
    }
    LoRa.onReceive(onReceive1);
    Serial1.printf("LoRa Init\n");
    lora_state = 0;
}


void onReceive1(int packetSize) // ! from isr
{
    lora_state = LoRa.read();
    while (LoRa.available()){
        LoRa.read();
    }
    rcv_flg=1;
}

void LoRa_send()
{
    LoRa.beginPacket();
    #ifdef RCV
    LoRa.print("RX GOOD");
    #else
    LoRa.write(!digitalRead(B1));
    #endif
    LoRa.endPacket();
    Serial.println("Packet sent");
    LoRa.receive();
}

void send_at_interval() {
    static unsigned long previousMillis = 0;
    // Check if it's time to execute the action
    if (millis() - previousMillis >= 350) {
        // Save the last time the action was executed
        previousMillis = millis();
        // Call your function here
        LoRa_send();
    }
}

int lora_update(){
    static unsigned long previousRCV = 0;
    if(rcv_flg){
        rcv_flg = 0;
        previousRCV = millis();
    }
    else if(millis() - previousRCV > TIMEOUT){
        lora_state = 0;
    }
}
