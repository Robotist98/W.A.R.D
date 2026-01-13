#include "adafruit_telemetry.h"

// ===== UniversalPacker Implementation =====

UniversalPacker::UniversalPacker()
{
    // Initialize packer state and zero the buffer
    clear();
}

void UniversalPacker::clear()
{
    // Reset the length counter and clear the internal 8-byte buffer.
    // Clearing the buffer is optional for functionality, but it makes
    // debugging and transmitted padded frames deterministic.
    m_size = 0;
    memset(data, 0, sizeof(data));
}

bool UniversalPacker::addUint8(uint8_t value)
{
    // Append a single byte if there is space.
    if (m_size >= CAN_MAX_DLC) return false;
    data[m_size++] = value;
    return true;
}

bool UniversalPacker::addUint16(uint16_t value)
{
    // Ensure two bytes available, then write in big-endian (MSB first).
    if (m_size + 2 > CAN_MAX_DLC) return false;
    size_t i = m_size;
    data[i++] = (value >> 8) & 0xFF;
    data[i++] = value & 0xFF;
    m_size = i;
    return true;
}

bool UniversalPacker::addUint32(uint32_t value)
{
    // Ensure four bytes available, then write 32-bit value as big-endian.
    if (m_size + 4 > CAN_MAX_DLC) return false;
    size_t i = m_size;
    data[i++] = (value >> 24) & 0xFF;
    data[i++] = (value >> 16) & 0xFF;
    data[i++] = (value >> 8) & 0xFF;
    data[i++] = value & 0xFF;
    m_size = i;
    return true;
}

bool UniversalPacker::addUint64(uint64_t value)
{
    // Ensure eight bytes available, then write 64-bit value as big-endian.
    if (m_size + 8 > CAN_MAX_DLC) return false;
    size_t i = m_size;
    data[i++] = (value >> 56) & 0xFF;
    data[i++] = (value >> 48) & 0xFF;
    data[i++] = (value >> 40) & 0xFF;
    data[i++] = (value >> 32) & 0xFF;
    data[i++] = (value >> 24) & 0xFF;
    data[i++] = (value >> 16) & 0xFF;
    data[i++] = (value >> 8) & 0xFF;
    data[i++] = value & 0xFF;
    m_size = i;
    return true;
}

bool UniversalPacker::addString(const char* str)
{
    // Append a null-terminated string if there is space.
    size_t len = strlen(str);
    if (m_size + len + 1 > CAN_MAX_DLC) return false; // +1 for null terminator
    memcpy(&data[m_size], str, len);
    m_size += len;
    return true;
}

const uint8_t* UniversalPacker::getData() const
{
    // Return pointer to internal buffer. Caller must not modify contents.
    return data;
}

size_t UniversalPacker::getSize() const
{
    // Current number of bytes appended to the packer buffer.
    return m_size;
}

bool UniversalPacker::unpackUint8(const uint8_t* data, size_t offset, uint8_t& value, size_t sizeLimit)
{
    // Validate offset then read a single byte.
    if (offset >= sizeLimit) return false;
    value = data[offset];
    return true;
}

bool UniversalPacker::unpackUint16(const uint8_t* data, size_t offset, uint16_t& value, size_t sizeLimit)
{
    // Ensure two bytes available, then reconstruct big-endian 16-bit value.
    if (offset + 2 > sizeLimit) return false;
    value = (uint16_t(data[offset]) << 8) | uint16_t(data[offset + 1]);
    return true;
}

bool UniversalPacker::unpackUint32(const uint8_t* data, size_t offset, uint32_t& value, size_t sizeLimit)
{
    // Ensure four bytes available, then reconstruct big-endian 32-bit value.
    if (offset + 4 > sizeLimit) return false;
    value = (uint32_t(data[offset]) << 24) | (uint32_t(data[offset + 1]) << 16) |
            (uint32_t(data[offset + 2]) << 8) | uint32_t(data[offset + 3]);
    return true;
}

bool UniversalPacker::unpackUint64(const uint8_t* data, size_t offset, uint64_t& value, size_t sizeLimit)
{
    // Ensure eight bytes available, then reconstruct big-endian 64-bit value.
    if (offset + 8 > sizeLimit) return false;
    value = (uint64_t(data[offset]) << 56) | (uint64_t(data[offset + 1]) << 48) |
            (uint64_t(data[offset + 2]) << 40) | (uint64_t(data[offset + 3]) << 32) |
            (uint64_t(data[offset + 4]) << 24) | (uint64_t(data[offset + 5]) << 16) |
            (uint64_t(data[offset + 6]) << 8) | uint64_t(data[offset + 7]);
    return true;
}

bool UniversalPacker::unpackString(const uint8_t* data, size_t offset, char* str, size_t strSize, size_t sizeLimit)
{
    // Copy up to strSize-1 bytes from data (starting at offset), or until sizeLimit is reached.
    // Do not look for a null terminator; just copy up to strSize-1 bytes.
    if (offset >= sizeLimit || strSize == 0) return false;
    size_t maxCopy = strSize - 1;
    size_t available = sizeLimit > offset ? sizeLimit - offset : 0;
    size_t toCopy = (available < maxCopy) ? available : maxCopy;
    memcpy(str, data + offset, toCopy);
    str[toCopy] = '\0'; // Always null-terminate
    return true;
}

// ===== Telemetry Implementation =====

Telemetry::Telemetry(uint8_t mcpPin)
  : Adafruit_MCP2515(mcpPin),
    m_telemetryInterval(500),   // ms
    m_receivedSize(0),
    m_mcpPin(mcpPin)
{
    // Constructor stores the chip-select pin used for the MCP2515 instance.
}

Telemetry::Telemetry(uint8_t mcpPin, uint8_t miso, uint8_t mosi, uint8_t sck)
  : Adafruit_MCP2515(mcpPin, miso, mosi, sck),
    m_telemetryInterval(500),   // ms
    m_receivedSize(0),
    m_mcpPin(mcpPin)
{
    // Constructor stores the chip-select pin used for the MCP2515 instance.
}

int Telemetry::begin(long baudRate)
{
    long rate = baudRate;

    // Clamp to the set of rates previously supported by the mcp_can wrapper.
    switch (baudRate)
    {
        case 1000000: rate = 1000000; break;
        case 500000:  rate = 500000;  break;
        case 250000:  rate = 250000;  break;
        case 125000:  rate = 125000;  break;
        case 100000:  rate = 100000;  break;
        case 50000:   rate = 50000;   break;
        default:      rate = 500000;  break;
    }

    // Adafruit driver returns non-zero on success.
    return Adafruit_MCP2515::begin(rate) != 0;
}

uint8_t Telemetry::getMcpPin() const
{
    // Return the configured SPI chip-select pin number.
    return m_mcpPin;
}

void Telemetry::setTelemetryInterval(uint16_t interval)
{
    // Allow users to adjust a stored telemetry interval (ms). This value is
    // not used by the library itself but available for higher-level logic.
    m_telemetryInterval = interval;
}
uint16_t Telemetry::getTelemetryInterval() const
{
    return m_telemetryInterval;
}

uint32_t Telemetry::getLastReceivedId() const
{
    // Return the CAN ID of the last successfully received frame.
    return m_lastReceivedId;
}

void Telemetry::send(uint32_t id, const UniversalPacker& packer, bool padded)
{
    size_t len = packer.getSize();
    if (len > UniversalPacker::CAN_MAX_DLC) len = UniversalPacker::CAN_MAX_DLC;

    const int canId = static_cast<int>(id & 0x7FF);
    const size_t sendLen = padded ? UniversalPacker::CAN_MAX_DLC : len;

    if (!Adafruit_MCP2515::beginPacket(canId, static_cast<int>(sendLen))) {
        return;
    }

    if (!padded) {
        Adafruit_MCP2515::write(packer.getData(), sendLen);
    } else {
        uint8_t buffer[UniversalPacker::CAN_MAX_DLC] = {0};
        memcpy(buffer, packer.getData(), len);
        Adafruit_MCP2515::write(buffer, sendLen);
    }

    Adafruit_MCP2515::endPacket();
}

// ===== First receive() - no filter (any frame) =====
bool Telemetry::receive()
{
    int packetLength = Adafruit_MCP2515::parsePacket();
    if (packetLength <= 0) {
        return false;
    }

    if (packetLength > static_cast<int>(UniversalPacker::CAN_MAX_DLC)) {
        packetLength = UniversalPacker::CAN_MAX_DLC;
    }

    size_t i = 0;
    while (i < static_cast<size_t>(packetLength) && Adafruit_MCP2515::available()) {
        int byteVal = Adafruit_MCP2515::read();
        if (byteVal < 0) break;
        m_receivedData[i++] = static_cast<uint8_t>(byteVal);
    }

    m_receivedSize = i;
    m_lastReceivedId = static_cast<uint32_t>(Adafruit_MCP2515::packetId());
    return true;
}

// ===== Overloaded receive(filterId) - filtered by CAN ID =====
bool Telemetry::receive(uint32_t filterId)
{
    int packetLength = Adafruit_MCP2515::parsePacket();
    if (packetLength <= 0) {
        return false;
    }

    uint32_t rxId = static_cast<uint32_t>(Adafruit_MCP2515::packetId());
    if ((rxId & 0x7FF) != (filterId & 0x7FF)) {
        while (Adafruit_MCP2515::available()) {
            Adafruit_MCP2515::read();
        }
        return false;
    }

    if (packetLength > static_cast<int>(UniversalPacker::CAN_MAX_DLC)) {
        packetLength = UniversalPacker::CAN_MAX_DLC;
    }

    size_t i = 0;
    while (i < static_cast<size_t>(packetLength) && Adafruit_MCP2515::available()) {
        int byteVal = Adafruit_MCP2515::read();
        if (byteVal < 0) break;
        m_receivedData[i++] = static_cast<uint8_t>(byteVal);
    }

    m_receivedSize = i;
    m_lastReceivedId = rxId;
    return true;
}

bool Telemetry::getUint8(size_t offset, uint8_t& value)
{
    // Read a uint8 from the internal receive buffer at `offset`.
    return UniversalPacker::unpackUint8(m_receivedData, offset, value, m_receivedSize);
}

bool Telemetry::getUint16(size_t offset, uint16_t& value)
{
    return UniversalPacker::unpackUint16(m_receivedData, offset, value, m_receivedSize);
}

bool Telemetry::getUint32(size_t offset, uint32_t& value)
{
    // Read a uint32 from the internal receive buffer at `offset`.
    return UniversalPacker::unpackUint32(m_receivedData, offset, value, m_receivedSize);
}

bool Telemetry::getUint64(size_t offset, uint64_t& value)
{
    // Read a uint64 from the internal receive buffer at `offset`.
    return UniversalPacker::unpackUint64(m_receivedData, offset, value, m_receivedSize);
}

bool Telemetry::getString(size_t offset, char* str, size_t strSize)
{
    // Extract a null-terminated string from the internal receive buffer.
    return UniversalPacker::unpackString(m_receivedData, offset, str, strSize, m_receivedSize);
}
// End of telemetry.cpp