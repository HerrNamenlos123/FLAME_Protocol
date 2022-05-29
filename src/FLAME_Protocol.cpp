
#include "FLAME_Protocol.h"

#if defined(ARDUINO) && ARDUINO >= 100      // Only for IPAddress to string
#include "Ethernet.h"
#endif

namespace FLAME_Protocol {

    FLAME_Wrapper wrapper = FLAME_Wrapper_init_zero;
    FLAME_ControlPacket toMCU = FLAME_ControlPacket_init_zero;
    FLAME_ReviewPacket toPC = FLAME_ReviewPacket_init_zero;
    uint8_t buffer[FLAME_PROTOCOL_BUFFER_SIZE];

    uint32_t mcu_ip = 0;
    uint32_t pc_ip = 0;
    uint32_t lastControlPacket = 0;
    uint32_t lastReview = 0;

    uint16_t CRC16(uint8_t* data, size_t len) {
        uint16_t crc = 0x1337;
        static const uint16_t table[] = {
          0x0000, 0x3D65, 0x7ACA, 0x47AF, 0xF594, 0xC8F1, 0x8F5E, 0xB23B, 0xD64D, 0xEB28, 0xAC87, 0x91E2, 0x23D9, 0x1EBC, 0x5913, 0x6476,
          0x91FF, 0xAC9A, 0xEB35, 0xD650, 0x646B, 0x590E, 0x1EA1, 0x23C4, 0x47B2, 0x7AD7, 0x3D78, 0x001D, 0xB226, 0x8F43, 0xC8EC, 0xF589,
          0x1E9B, 0x23FE, 0x6451, 0x5934, 0xEB0F, 0xD66A, 0x91C5, 0xACA0, 0xC8D6, 0xF5B3, 0xB21C, 0x8F79, 0x3D42, 0x0027, 0x4788, 0x7AED,
          0x8F64, 0xB201, 0xF5AE, 0xC8CB, 0x7AF0, 0x4795, 0x003A, 0x3D5F, 0x5929, 0x644C, 0x23E3, 0x1E86, 0xACBD, 0x91D8, 0xD677, 0xEB12,
          0x3D36, 0x0053, 0x47FC, 0x7A99, 0xC8A2, 0xF5C7, 0xB268, 0x8F0D, 0xEB7B, 0xD61E, 0x91B1, 0xACD4, 0x1EEF, 0x238A, 0x6425, 0x5940,
          0xACC9, 0x91AC, 0xD603, 0xEB66, 0x595D, 0x6438, 0x2397, 0x1EF2, 0x7A84, 0x47E1, 0x004E, 0x3D2B, 0x8F10, 0xB275, 0xF5DA, 0xC8BF,
          0x23AD, 0x1EC8, 0x5967, 0x6402, 0xD639, 0xEB5C, 0xACF3, 0x9196, 0xF5E0, 0xC885, 0x8F2A, 0xB24F, 0x0074, 0x3D11, 0x7ABE, 0x47DB,
          0xB252, 0x8F37, 0xC898, 0xF5FD, 0x47C6, 0x7AA3, 0x3D0C, 0x0069, 0x641F, 0x597A, 0x1ED5, 0x23B0, 0x918B, 0xACEE, 0xEB41, 0xD624,
          0x7A6C, 0x4709, 0x00A6, 0x3DC3, 0x8FF8, 0xB29D, 0xF532, 0xC857, 0xAC21, 0x9144, 0xD6EB, 0xEB8E, 0x59B5, 0x64D0, 0x237F, 0x1E1A,
          0xEB93, 0xD6F6, 0x9159, 0xAC3C, 0x1E07, 0x2362, 0x64CD, 0x59A8, 0x3DDE, 0x00BB, 0x4714, 0x7A71, 0xC84A, 0xF52F, 0xB280, 0x8FE5,
          0x64F7, 0x5992, 0x1E3D, 0x2358, 0x9163, 0xAC06, 0xEBA9, 0xD6CC, 0xB2BA, 0x8FDF, 0xC870, 0xF515, 0x472E, 0x7A4B, 0x3DE4, 0x0081,
          0xF508, 0xC86D, 0x8FC2, 0xB2A7, 0x009C, 0x3DF9, 0x7A56, 0x4733, 0x2345, 0x1E20, 0x598F, 0x64EA, 0xD6D1, 0xEBB4, 0xAC1B, 0x917E,
          0x475A, 0x7A3F, 0x3D90, 0x00F5, 0xB2CE, 0x8FAB, 0xC804, 0xF561, 0x9117, 0xAC72, 0xEBDD, 0xD6B8, 0x6483, 0x59E6, 0x1E49, 0x232C,
          0xD6A5, 0xEBC0, 0xAC6F, 0x910A, 0x2331, 0x1E54, 0x59FB, 0x649E, 0x00E8, 0x3D8D, 0x7A22, 0x4747, 0xF57C, 0xC819, 0x8FB6, 0xB2D3,
          0x59C1, 0x64A4, 0x230B, 0x1E6E, 0xAC55, 0x9130, 0xD69F, 0xEBFA, 0x8F8C, 0xB2E9, 0xF546, 0xC823, 0x7A18, 0x477D, 0x00D2, 0x3DB7,
          0xC83E, 0xF55B, 0xB2F4, 0x8F91, 0x3DAA, 0x00CF, 0x4760, 0x7A05, 0x1E73, 0x2316, 0x64B9, 0x59DC, 0xEBE7, 0xD682, 0x912D, 0xAC48
        };

        if (data == NULL)
            return 0xffff;

        crc &= 0xffff;
        while (len--)
            crc = (crc << 8) ^ table[((crc >> 8) ^ *data++)];

        return crc;
    }






	// ============================================
	// =====    PROTOCOL PARSING FUNCTIONS    =====
	// ============================================

    void discoveryPacketReceived(uint32_t sourceIP) {      // On MCU

#if defined(ARDUINO) && ARDUINO >= 100		// if Arduino
        IPAddress addr(sourceIP);
        Serial.print("Discovery packet received: ");
        addr.printTo(Serial);
        Serial.println();
#else                                       // Non-Arduino
        return;
#endif

        wrapper = FLAME_Wrapper_init_zero;
        wrapper.discoveryResponse.ipAddress = getLocalIP();
        wrapper.has_discoveryResponse = true;

        pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
        if (!pb_encode(&stream, FLAME_Wrapper_fields, &wrapper)) {
            DEBUG_PRINTF("Failed to encode packet!");
            return;
        }
        writeUDP(sourceIP, FLAME_PROTOCOL_UDP_TARGET_PORT, buffer, stream.bytes_written);
    }

	void packetReceived(const uint8_t* packet, uint8_t length, uint32_t sourceIP, uint16_t sourcePort) {
        
        wrapper = FLAME_Wrapper_init_zero;
        pb_istream_t stream = pb_istream_from_buffer(packet, length);
        if (!pb_decode(&stream, FLAME_Wrapper_fields, &wrapper)) {
            toPC.badPackets++;
            return;
        }

        if (wrapper.has_discoveryResponse) {        // On PC
            mcu_ip = sourceIP;
        }

        if (wrapper.has_reviewPacket) {             // On PC
            toPC = wrapper.reviewPacket;
        }

        if (wrapper.has_discoveryPacket) {      // On MCU
            discoveryPacketReceived(sourceIP);
        }

        if (wrapper.has_controlPacket) {        // On MCU
            lastControlPacket = getMicros();
            pc_ip = sourceIP;
            toMCU = wrapper.controlPacket;
        }
	}

    void sendReviewPacket() {   // To PC

        if (pc_ip == 0)
            return;

        wrapper = FLAME_Wrapper_init_zero;
        wrapper.reviewPacket = toPC;
        wrapper.has_reviewPacket = true;

        pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
        if (!pb_encode(&stream, FLAME_Wrapper_fields, &wrapper)) {
            DEBUG_PRINTF("Failed to encode packet!");
            return;
        }
        writeUDP(pc_ip, FLAME_PROTOCOL_UDP_TARGET_PORT, buffer, stream.bytes_written);
    }

    void sendDiscoveryPacket(uint32_t broadcastIP) {    // To MCU

        if (broadcastIP == 0)
            return;

        wrapper = FLAME_Wrapper_init_zero;
        wrapper.has_discoveryPacket = true;

        pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
        if (!pb_encode(&stream, FLAME_Wrapper_fields, &wrapper)) {
            DEBUG_PRINTF("Failed to encode packet!");
            return;
        }
        writeUDPBroadcast(broadcastIP, FLAME_PROTOCOL_UDP_TARGET_PORT, buffer, stream.bytes_written);
    }

    void sendControlPacket() {    // To MCU

        if (mcu_ip == 0)
            return;

        wrapper = FLAME_Wrapper_init_zero;
        wrapper.controlPacket = toMCU;
        wrapper.has_controlPacket = true;

        pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
        if (!pb_encode(&stream, FLAME_Wrapper_fields, &wrapper)) {
            DEBUG_PRINTF("Failed to encode packet!");
            return;
        }
        writeUDP(mcu_ip, FLAME_PROTOCOL_UDP_TARGET_PORT, buffer, stream.bytes_written);
    }

	void update() {

		uint32_t now = getMicros();
		if (now - lastReview >= FLAME_PROTOCOL_REVIEW_CYCLE_TIME) {
    	    lastReview = now;

    	    if (now - lastControlPacket < FLAME_PROTOCOL_CONTROL_PACKET_TIMEOUT) {
                sendReviewPacket();
    	    }
    	    else {
    	        if (!toPC.safetyMode) {
    	            DEBUG_PRINTF("Control packet timeout, entering safety mode");
    	        }
                toPC.safetyMode = true;
    	    }

            if (toPC.safetyMode && toMCU.clearSafetyMode) {
                DEBUG_PRINTF("Safety mode disabled by controller");
                toMCU.clearSafetyMode = false;
                toPC.safetyMode = false;
            }
    	}
	}
}
