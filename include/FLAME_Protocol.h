#pragma once

// client = Arduino Mega (robot arm)
// user = controller of the robot arm (e.g. PC)

// The control packet must be sent to the client periodically, preferrably at around 500Hz.
// The review packet is continuously sent back to the user at 500Hz and is completely independent.
// Packets are always sent to the target port 22500. After a client is identified using the discovery packet, 
// streaming can be started immediately. As soon as the client receives a control packet, the review packet 
// starts to be streamed back to the sender of the last control packet.
// It continues to be streamed for 1 second after the last control packet was received -> Thus, minimum control
// frequency before timeout it 1Hz
//
// === Control packet ===
//
// Byte  0-3  -> Desired position of axis 1
// Byte  4-7  -> Desired position of axis 2
// Byte  8-11 -> Desired position of axis 3
// Byte 12-15 -> Desired position of axis 4
// Byte  16	  -> Endpoint ID of the additional data, 0 for no operation
// Byte 17-20 -> 4-byte value, datatype according to the endpoint id
// Byte 21-22 -> CRC16 over bytes 0-21
//
// The first 4 variables are the desired positions of the axis 1-4 in degrees from the reference mark.
// The endpoint ID specifies an endpoint, followed by 4 bytes, representing any data type. The endpoint ID can be
// cycled through all necessary endpoints, so that the desired positions are constantly updated and all other 
// endpoints every n-th cycle.
//
// === Review packet ===
//
// Byte  0	  -> Endpoint ID of the sent data, 0 for no operation
// Byte 1-4   -> 4-byte value, datatype according to the endpoint id
// Byte 5-6   -> CRC16 over bytes 0-4
//
// The Arduino sends this packet to the user continuously. The endpoint ID is cycled through all available endpoints,
// so every endpoint is updated every n-th cycle.
//

// Discovery Packet
// Byte 0 -> 0x42

// Discovery Response
// Byte 0-3 -> IPv4 address of the client
// Byte 4-5 -> CRC16 over bytes 0-3

#include "stdint.h"

#define FLAME_PROTOCOL_UDP_TARGET_PORT 22500
#define FLAME_PROTOCOL_BUFFER_SIZE 256
#define FLAME_PROTOCOL_REVIEW_CYCLE_TIME (1000000UL / 100)	// (... / Hz)
#define FLAME_PROTOCOL_CONTROL_PACKET_TIMEOUT (1000UL * 100)	// (... * ms)

#define FLAME_DEBUG

#if defined(ARDUINO) && ARDUINO >= 100		// if Arduino
	#include "Arduino.h"
	#ifdef FLAME_DEBUG
		//#define DEBUG_PRINT(...) Serial.println(__VA_ARGS__)
		#define DEBUG_PRINTF(fmt, ...) { char buffer[64]; sprintf(buffer, fmt "\n", ##__VA_ARGS__); Serial.print(buffer); }
	#else
		#define DEBUG_PRINTF()
	#endif
#else										// if not Arduino
	#include <cstring>
	#include <stdio.h>
	#ifdef FLAME_DEBUG
		#define DEBUG_PRINTF(fmt, ...) printf(fmt "\n", ##__VA_ARGS__)
	#else
		#define DEBUG_PRINTF()
	#endif
#endif

#include "pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "FLAME.pb.h"


// These functions must be defined by the user
uint32_t getMicros();
uint32_t getLocalIP();
void writeUDP(uint32_t targetIP, uint16_t targetPort, uint8_t* data, uint8_t length);
void writeUDPBroadcast(uint32_t targetIP, uint16_t targetPort, uint8_t* data, uint8_t length);

namespace FLAME_Protocol {

	extern FLAME_ControlPacket toMCU;
	extern FLAME_ReviewPacket toPC;
	extern uint32_t mcu_ip;
	extern uint32_t pc_ip;

	void packetReceived(const uint8_t* packet, uint8_t length, uint32_t sourceIP, uint16_t sourcePort);

	void sendDiscoveryPacket(uint32_t broadcastIP);
	void sendControlPacket();

	void update();

}
