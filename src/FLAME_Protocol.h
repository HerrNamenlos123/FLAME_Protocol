#pragma once

// 4 * 4 byte fix 
// id 
// 4 byte 
// 2 byte checksum CRC16 ODRIVE
// = 23 bytes for standard packet

#include "stdint.h"

#ifdef _WIN32
#include <cstring>
#else
#include "Arduino.h"
#endif

#define FLAME_PROTOCOL_STANDARD_PACKET_LENGTH 23

namespace FLAME_Protocol {

	struct Packet {
		uint8_t data[FLAME_PROTOCOL_STANDARD_PACKET_LENGTH];
	};

	struct PacketData {
		float axis1=0;
		float axis2=0;
		float axis3=0; 
		float axis4=0;
		uint8_t id=0;
		uint32_t additional=0;
	};

	void generatePacket(FLAME_Protocol::Packet* packet, FLAME_Protocol::PacketData* pd);

	bool parsePacket(FLAME_Protocol::Packet* packet, FLAME_Protocol::PacketData* pd);

	uint16_t CRC16(uint8_t* data, size_t len);
}
