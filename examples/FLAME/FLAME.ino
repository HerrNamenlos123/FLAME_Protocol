/*
#include "timer.h"
#include "Endpoints.h"
#include "Utility.h"
#include "ODrive.h"
#include "Process.h"

#include <SPI.h>
#include <Ethernet.h>
#include <FLAME_Protocol.h>

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
uint8_t udpBuffer[FLAME_PROTOCOL_MAX_PACKET_LENGTH];
EthernetUDP server;

uint32_t lastCycle = 0;
uint32_t cycleTime = 10000;

FLAME_Protocol::FLAME_Instance flameInstance;
extern ODrive odrv0;
extern ODrive odrv1;

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
        FLAME_Protocol::PacketReceived(&flameInstance, udpBuffer, packetSize, server.remoteIP());
    }

    // Send the Review packet
    FLAME_Protocol::UpdateReviewStream(&flameInstance);
}

void setup() {
    Serial.begin(115200);
    odrv0.serialBegin();
    odrv1.serialBegin();

    Serial.println("Connecting...");
    
    Ethernet.init(53);

    bool staticIP = true;
    if (staticIP) {
        IPAddress ip(192, 168, 25, 2);
        Ethernet.begin(mac, ip);
    }
    else {
        if (!Ethernet.begin(mac)) {
            Serial.println("Failed to get DHCP address");
            while(true);
        }
    }
    if (Ethernet.hardwareStatus() == EthernetNoHardware || Ethernet.linkStatus() == LinkOFF) {
        Serial.println("Failed to connect");
        while(true);
    };

    Serial.print("Connected with IP: ");
    Serial.println(Ethernet.localIP());
    
    server.begin(flameInstance.receiverPort);
}

*/

#include "Arduino.h"
#include "ODriveBackend.h"
#include "process.h"

void setup() {
    Serial.begin(115200);
    Serial.println("Starting");

    setupODrives();
}

void loop() {
    
    updateProcess();
    updateODrives();

}
