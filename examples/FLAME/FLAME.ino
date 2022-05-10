
#include "Arduino.h"
#include "ODriveBackend.h"
#include "process.h"

#include <SPI.h>
#include <Ethernet.h>
#include <FLAME_Protocol.h>

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
uint8_t udpBuffer[256];
EthernetUDP server;

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
        FLAME_Protocol::packetReceived(udpBuffer, packetSize, server.remoteIP(), server.remotePort());
    }

    // Send the Review packet
    FLAME_Protocol::update();
}

void setup() {
    Serial.begin(115200);
    Serial.println("Connecting...");
    
    Ethernet.init(53);

    bool staticIP = true;
    if (staticIP) {
        IPAddress ip(10, 0, 0, 50);
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
    
    server.begin(FLAME_PROTOCOL_UDP_TARGET_PORT);

    setupODrives();
    startProcess();
}

void loop() {
    
    updateProcess();
    updateODrives();
    updateUDP();

}
