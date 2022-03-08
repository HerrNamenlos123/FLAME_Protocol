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

#define FLAME_PROTOCOL_CONTROL_BYTE 0x42
#define FLAME_PROTOCOL_CONTROL_PACKET_LENGTH 23
#define FLAME_PROTOCOL_REVIEW_PACKET_LENGTH 7
#define FLAME_PROTOCOL_DISCOVERY_PACKET_LENGTH 1
#define FLAME_PROTOCOL_DISCOVERY_RESPONSE_LENGTH 6
#define FLAME_PROTOCOL_MAX_PACKET_LENGTH FLAME_PROTOCOL_CONTROL_PACKET_LENGTH
#define FLAME_PROTOCOL_UDP_TARGET_PORT 22500

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


// These functions must be defined by the user
uint32_t getMicros();
uint32_t getLocalIP();
void writeUDP(uint32_t targetIP, uint16_t targetPort, uint8_t* data, uint8_t length);

namespace FLAME_Protocol {




	// ================================
	// =====    CONTROL PACKET    =====
	// ================================

	struct ControlPacket {
		float axis1 = 0;
		float axis2 = 0;
		float axis3 = 0;
		float axis4 = 0;
		uint8_t id = 0;
		uint32_t additional = 0;
	};

	/// <summary>
	/// Buffer must be 23 bytes large!
	/// </summary>
	void generatePacket(FLAME_Protocol::ControlPacket* controlPacket, uint8_t* buffer);

	/// <summary>
	/// Buffer must be 23 bytes large!
	/// </summary>
	bool parsePacket(FLAME_Protocol::ControlPacket* controlPacket, uint8_t* buffer);








	// ===============================
	// =====    Review PACKET    =====
	// ===============================

	struct ReviewPacket {
		uint8_t id = 0;
		uint32_t data = 0;
	};

	/// <summary>
	/// Buffer must be 7 bytes large!
	/// </summary>
	void generatePacket(FLAME_Protocol::ReviewPacket* reviewPacket, uint8_t* buffer);

	/// <summary>
	/// Buffer must be 7 bytes large!
	/// </summary>
	bool parsePacket(FLAME_Protocol::ReviewPacket* reviewPacket, uint8_t* buffer);








	// ==================================
	// =====    DISCOVERY PACKET    =====
	// ==================================

	/// <summary>
	/// Buffer must be 1 byte large!
	/// </summary>
	void generatePacket(uint8_t* buffer);

	/// <summary>
	/// Buffer must be 1 byte large!
	/// </summary>
	bool parsePacket(uint8_t* buffer);








	// ===========================================
	// =====    DISCOVERY RESPONSE PACKET    =====
	// ===========================================

	struct DiscoveryResponse {
		uint32_t ipAddress = 0;
	};

	/// <summary>
	/// Buffer must be 6 bytes large!
	/// </summary>
	void generatePacket(FLAME_Protocol::DiscoveryResponse* discoveryResponse, uint8_t* buffer);

	/// <summary>
	/// Buffer must be 6 bytes large!
	/// </summary>
	bool parsePacket(FLAME_Protocol::DiscoveryResponse* discoveryResponse, uint8_t* buffer);







	// =======================================
	// =====    FLAME INSTANCE STRUCT    =====
	// =======================================

	template<typename T>
	class Endpoint {
		uint32_t* location;
	public:
		Endpoint(uint32_t* location) : location(location) {}
		operator T() {
			T value = 0;
			memcpy(&value, location, sizeof(T));
			return value;
		}
		void operator=(T value) {
			*location = 0;
			memcpy(location, &value, sizeof(T));
		}
	};

	struct FLAME_Instance {
		
		ControlPacket controlPacket;
		ReviewPacket reviewPacket;
		DiscoveryResponse discoveryResponse;

		uint32_t registers[19];

		Endpoint<float> desiredAxis1 = registers + 0;
		Endpoint<float> desiredAxis2 = registers + 1;
		Endpoint<float> desiredAxis3 = registers + 2;
		Endpoint<float> desiredAxis4 = registers + 3;

		Endpoint<float> actualAxis1 = registers + 4;
		Endpoint<float> actualAxis2 = registers + 5;
		Endpoint<float> actualAxis3 = registers + 6;
		Endpoint<float> actualAxis4 = registers + 7;

		Endpoint<bool> odrive0Axis0Error = registers + 8;
		Endpoint<bool> odrive0Axis1Error = registers + 9;
		Endpoint<bool> odrive0Error = registers + 10;	
		Endpoint<bool> odrive1Axis0Error = registers + 11;
		Endpoint<bool> odrive1Axis1Error = registers + 12;
		Endpoint<bool> odrive1Error = registers + 13;	

		Endpoint<bool> safetyMode = registers + 14;
		Endpoint<bool> odrv0ErrorState = registers + 15;
		Endpoint<bool> odrv0ClearErrors = registers + 16;
		Endpoint<bool> odrv1ErrorState = registers + 17;
		Endpoint<bool> odrv1ClearErrors = registers + 18;




		uint32_t badPackets = 0;
		uint32_t controllerIP = 0;
		uint32_t controllerPort = FLAME_PROTOCOL_UDP_TARGET_PORT;
		uint32_t receiverIP = 0;
		uint32_t receiverPort = FLAME_PROTOCOL_UDP_TARGET_PORT;
		uint32_t lastControlPacket = 0;
		uint32_t controlPacketTimeout = 100000;      // us
		uint32_t lastReview = 0;
		uint32_t reviewCycleTime = 500;       // us
		uint32_t reviewPacketCount = 0;

		uint8_t packetBuffer[FLAME_PROTOCOL_MAX_PACKET_LENGTH];

	};






	// ============================================
	// =====    PROTOCOL PARSING FUNCTIONS    =====
	// ============================================

	void controlPacketReceived(FLAME_Instance* flame);

	void discoveryPacketReceived(FLAME_Instance* flame);

	void PacketReceived(FLAME_Instance* flame, uint8_t* buffer, uint8_t length, uint32_t controllerIP);

	void makeReviewPacket(FLAME_Instance* flame);

	void UpdateReviewStream(FLAME_Instance* flame);

}
