// 4 * 4 byte fix 
// id 
// 4 byte 
// 2 byte checksum CRC16 ODRIVE
#pragma once
#ifdef _WIN32
#include <cstring>
#else
#include "Arduino.h"
#endif

#include "stdint.h"
namespace FLAME_Protocol {
	struct Packet {
		uint8_t data[23];
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


