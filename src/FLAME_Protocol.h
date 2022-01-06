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

#ifdef _WIN32
#include <cstring>
#else
#include "Arduino.h"
#endif

#define FLAME_PROTOCOL_CONTROL_BYTE 0x42
#define FLAME_PROTOCOL_CONTROL_PACKET_LENGTH 23
#define FLAME_PROTOCOL_REVIEW_PACKET_LENGTH 7
#define FLAME_PROTOCOL_DISCOVERY_PACKET_LENGTH 1
#define FLAME_PROTOCOL_DISCOVERY_RESPONSE_LENGTH 6

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

}
