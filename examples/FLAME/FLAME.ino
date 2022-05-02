
#include <SPI.h>
#include <Ethernet.h>
#include "FLAME_Protocol.h"

int cnt = 0;

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
uint8_t udpBuffer[FLAME_PROTOCOL_BUFFER_SIZE];
EthernetUDP server;

static inline size_t stack_size()
{
    return RAMEND - SP;
}

uint32_t getMicros() {
    return micros();
}

uint32_t getLocalIP() {
    return Ethernet.localIP();
}

void writeUDP(uint32_t targetIP, uint16_t targetPort, uint8_t* data, uint8_t length) {
    server.beginPacket(targetIP, targetPort);
    server.write(data, length);
    server.endPacket();
}

void updateUDP() {

    // Receive the packet
    int packetSize = server.parsePacket();
    if (packetSize != 0) {
        server.read(udpBuffer, sizeof(udpBuffer));
        //Serial.println("Packet received");
        FLAME_Protocol::packetReceived(udpBuffer, packetSize, server.remoteIP(), server.remotePort());
    }

    // Send the Review packet
    FLAME_Protocol::update();
}

void setup() {

    Serial.begin(115200);
    Serial.println("Connecting...");
    
    Ethernet.init(53);

    Ethernet.begin(mac, IPAddress(10, 0, 0, 50));
    //if (!Ethernet.begin(mac, IPAddress(10, 0, 0, 50))) {
    //    Serial.println("Failed to connect DHCP");
    //    while(true);
    //}

    if (Ethernet.hardwareStatus() == EthernetNoHardware || Ethernet.linkStatus() == LinkOFF) {
        Serial.println("Failed to connect, disconnected");
        while(true);
    }

    Serial.print("Connected with IP: ");
    Serial.println(Ethernet.localIP());
    
    server.begin(FLAME_PROTOCOL_UDP_TARGET_PORT);

}

void loop() {

    updateUDP();

}