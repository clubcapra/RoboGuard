#include "mylora.h"

#define TIMEOUT 500
#define FREQ_CANADA 904600E3
#define FREQ_EURO 869525E3

int rcv_flg = 0;
int lora_state = 0;

const int estop_pin = PA12;

void LoRa_init()
{
    SPI.setMISO(PB4);
    SPI.setMOSI(PB5);
    SPI.setSCLK(PB3);
    LoRa.setPins(PA15, PC7, PC8);
    LoRa.setTxPower(20);
    
    LoRa.setSyncWord(0xBE);
    if (!LoRa.begin(FREQ_CANADA)) // frequency
    {
        Serial1.println("\n[ERROR] LoRa init failed");
        abort();
    }
    LoRa.onReceive(onReceive1);
    LoRa.receive();
    Serial1.println("LoRa Init");
    lora_state = 0;
}


void onReceive1(int packetSize) // ! from isr
{
    Serial1.println("RCV");
    lora_state = LoRa.read();
    Serial1.print("STATE : ");
    Serial1.println(lora_state);
    while (LoRa.available()){
        LoRa.read();
    }
    rcv_flg=1;
    if(!lora_state){
        digitalWrite(estop_pin, lora_state);
    }
}

void LoRa_send()
{
    Serial1.println("SEND");
    LoRa.beginPacket();
    LoRa.write(1);
    LoRa.endPacket();
    Serial1.println("Packet sent");
    LoRa.receive();
}

void send_at_interval() {
    Serial1.println("send_at_interval");
    static unsigned long previousMillis = 0;
    if (millis() - previousMillis >= 350) {
        previousMillis = millis();
        LoRa_send();
    }
}

void lora_update(){
    static unsigned long previousRCV = 0;
    if(rcv_flg){
        rcv_flg = 0;
        LoRa_send();
        previousRCV = millis();
    }
    else if(millis() - previousRCV > TIMEOUT){
        lora_state = 0;
    }
}
