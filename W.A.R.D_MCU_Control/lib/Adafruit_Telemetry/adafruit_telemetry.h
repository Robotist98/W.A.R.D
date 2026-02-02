#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>
#include <Adafruit_MCP2515.h>
#include <SPI.h>

// ===== UniversalPacker Class =====
/**
 * @brief Simple packer for assembling CAN data payloads (big-endian byte order).
 *
 * UniversalPacker provides a lightweight API to sequentially append unsigned
 * integers (8/16/32-bit) into an internal 8-byte buffer suitable for CAN
 * frames (DLC up to 8). The buffer is stored in big-endian network order so
 * that multi-byte integers are transmitted MSB-first.
 * 
 * Usage example:
 *  UniversalPacker p;
 *  p.addUint8(1);
 *  p.addUint16(0x1234);
 *  p.addUint32(0x12345678);
 *  send(..., p);
 *
 */
class UniversalPacker {
public:
    /**
     * @brief Create an empty packer.
     *
     * The constructor clears the internal buffer and sets the size to zero.
     */
    UniversalPacker();

    /**
     * @brief Maximum CAN data length (DLC) supported by the packer.
     */
    static constexpr size_t CAN_MAX_DLC = 8;

    /**
     * @brief Reset the packer to empty state and zero the buffer.
     */
    void clear();

    /**
     * @brief Append an 8-bit unsigned integer to the buffer.
     * @param value The byte to append.
     * @return true if appended; false if there is no space.
     */
    bool addUint8(uint8_t value);

    /**
     * @brief Append a 16-bit unsigned integer (big-endian).
     * @param value The 16-bit integer to append.
     * @return true if appended; false if there is insufficient space.
     */
    bool addUint16(uint16_t value);

    /**
     * @brief Append a 32-bit unsigned integer (big-endian).
     * @param value The 32-bit integer to append.
     * @return true if appended; false if there is insufficient space.
     */
    bool addUint32(uint32_t value);


    /**
     * @brief Append a 64-bit unsigned integer (big-endian).
     * @param value The 64-bit integer to append.
     * @return true if appended; false if there is insufficient space.
     */
    bool addUint64(uint64_t value);

    /**
     * @brief Append a null-terminated string to the buffer.
     * @param str Pointer to the null-terminated string.
     * @return true if appended; false if there is insufficient space.
     */
    bool addString(const char* str);

    /**
     * @brief Get pointer to the internal 8-byte buffer.
     * @return Pointer to the immutable internal data buffer.
     */
    const uint8_t* getData() const;

    /**
     * @brief Get the number of bytes currently packed.
     * @return Number of valid bytes in the buffer (0..CAN_MAX_DLC).
     */
    size_t getSize() const;

    /**
     * @brief Unpack a single byte from a payload.
     * @param data Pointer to the received data buffer.
     * @param offset Byte offset within the buffer to read from (0-based).
     * @param[out] value Output reference that will receive the byte on success.
     * @param sizeLimit Number of valid bytes in `data` (bounds check).
     * @return true if the byte was read successfully; false if `offset` is out of bounds.
     */
    static bool unpackUint8(const uint8_t* data, size_t offset, uint8_t& value, size_t sizeLimit);

    /**
     * @brief Unpack a 16-bit unsigned integer (big-endian) from a payload.
     * @param data Pointer to the received data buffer.
     * @param offset Byte offset within the buffer where the 2-byte value starts.
     * @param[out] value Output reference that will receive the 16-bit value on success.
     * @param sizeLimit Number of valid bytes in `data` (bounds check).
     * @return true if two bytes were available and value was read; false otherwise.
     */
    static bool unpackUint16(const uint8_t* data, size_t offset, uint16_t& value, size_t sizeLimit);

    /**
     * @brief Unpack a 32-bit unsigned integer (big-endian) from a payload.
     * @param data Pointer to the received data buffer.
     * @param offset Byte offset within the buffer where the 4-byte value starts.
     * @param[out] value Output reference that will receive the 32-bit value on success.
     * @param sizeLimit Number of valid bytes in `data` (bounds check).
     * @return true if four bytes were available and value was read; false otherwise.
     */
    static bool unpackUint32(const uint8_t* data, size_t offset, uint32_t& value, size_t sizeLimit); 

    /**
     * @brief Unpack a 64-bit unsigned integer (big-endian) from a payload.
     * @param data Pointer to the received data buffer.
     * @param offset Byte offset within the buffer where the 8-byte value starts.
     * @param[out] value Output reference that will receive the 64-bit value on success.
     * @param sizeLimit Number of valid bytes in `data` (bounds check).
     * @return true if eight bytes were available and value was read; false otherwise.
     */
    static bool unpackUint64(const uint8_t* data, size_t offset, uint64_t& value, size_t sizeLimit);

    /**
     * @brief Unpack a null-terminated string from a payload.
     * @param data Pointer to the received data buffer.
     * @param offset Byte offset within the buffer where the string starts.
     * @param[out] str Pointer to a caller-provided buffer to receive the string.
     * @param strSize Size of the `str` buffer in bytes (including space for null).
     * @param sizeLimit Number of valid bytes in `data` (bounds check).
     * @return true if a null-terminated string was successfully extracted;
     * false if no null terminator was found within the available data.
     */
    static bool unpackString(const uint8_t* data, size_t offset, char* str, size_t strSize, size_t sizeLimit);

private:
    uint8_t data[8];
    /**
     * @brief Current count of valid bytes in `data` starting at index 0.
     *
     * Range: 0..CAN_MAX_DLC. Use `getSize()` to access this value.
     */
    size_t m_size;
};

// ===== Telemetry Class =====
/**
 * @brief Telemetry wrapper around Adafruit_MCP2515 providing helper methods
 * for sending/receiving CAN frames with the `UniversalPacker`.
 *
 * Telemetry inherits from `Adafruit_MCP2515` so it can call the underlying
 * driver methods directly. It stores the last received payload and ID;
 * callers can use `receive()` or `receive(filterId)` to populate the internal
 * buffer and then call the `getUint*` helpers to extract values.
 */
class Telemetry : public Adafruit_MCP2515 {
public:
    /**
     * @brief Construct a Telemetry wrapper bound to a given MCP2515 CS pin.
     * @param mcpPin The SPI chip-select pin used by the MCP2515 library.
     */
    explicit Telemetry(uint8_t mcpPin);

    explicit Telemetry(uint8_t mcpPin, uint8_t miso, uint8_t mosi, uint8_t sck);

    /**
     * @brief Initialize the MCP2515 hardware at the requested CAN bitrate.
     * @param baudRate Bitrate in bits/second (e.g. 500000).
     * @return 1 on success, 0 on failure.
     */
    int begin(long baudRate = 500E3);

    /**
     * @brief Store a telemetry interval value for higher-level logic.
     * @param interval Interval in milliseconds.
     */
    void setTelemetryInterval(uint16_t interval);

    /**
     * @brief Return the stored telemetry interval (ms).
     */
    uint16_t getTelemetryInterval() const;

    /**
     * @brief Get the CAN ID of the last received frame.
     */
    uint32_t getLastReceivedId() const;

    /**
     * @brief Get the configured SPI chip-select pin for the MCP2515.
     */
    uint8_t getMcpPin() const;

    /**
     * @brief Return the number of bytes from the last received frame.
     */
    size_t getReceivedSize() const { return m_receivedSize; }

    /**
     * @brief Send a CAN frame using the provided packer.
     * @param id Standard 11-bit CAN identifier.
     * @param packer UniversalPacker containing the payload.
     * @param padded If true, send a full 8-byte payload (padding with zeros).
     */
    void send(uint32_t id, const UniversalPacker& packer, bool padded = false);

    /**
     * @brief Check and read any available CAN frame into the internal buffer.
     * @return true when a frame was read and stored, false when no frame was available.
     */
    bool receive();

    /**
     * @brief Check and read an available CAN frame, but only accept frames
     * matching the provided standard 11-bit ID.
     * @param filterId The CAN ID to filter for (11-bit).
     * @return true when a matching frame was read, false otherwise.
     */
    bool receive(uint32_t filterId);
 
    /**
     * @brief Read an 8-bit value from the last received CAN payload stored
     * in `m_receivedData`.
     * @param offset Byte offset (0-based) to read from.
     * @param[out] value Output reference to receive the byte.
     * @return true if the byte was available; false otherwise.
     */
    bool getUint8(size_t offset, uint8_t& value);

    /**
     * @brief Read a 16-bit big-endian value from the last received payload.
     * @param offset Byte offset (0-based) where the 2-byte value starts.
     * @param[out] value Output reference to receive the 16-bit value.
     * @return true if two bytes were available and read; false otherwise.
     */
    bool getUint16(size_t offset, uint16_t& value);

    /**
     * @brief Read a 32-bit big-endian value from the last received payload.
     * @param offset Byte offset (0-based) where the 4-byte value starts.
     * @param[out] value Output reference to receive the 32-bit value.
     * @return true if four bytes were available and read; false otherwise.
     */
    bool getUint32(size_t offset, uint32_t& value);

    /**
     * @brief Read a 64-bit big-endian value from the last received payload.
     * @param offset Byte offset (0-based) where the 8-byte value starts.
     * @param[out] value Output reference to receive the 64-bit value.
     * @return true if eight bytes were available and read; false otherwise.
     */
    bool getUint64(size_t offset, uint64_t& value);

    /**
     * @brief Extract a null-terminated string from the last received payload.
     * @param offset Byte offset (0-based) where the string starts.
     * @param[out] str Pointer to a caller-provided buffer to receive the string.
     * @param strSize Size of the `str` buffer in bytes (including space for null).
     * @return true if a null-terminated string was successfully extracted;
     * false if no null terminator was found within the available data.
     */
    bool getString(size_t offset, char* str, size_t strSize);

    /**
     * @brief Read MCP2515 error flags register (EFLG).
     * @return Raw EFLG value.
     */
    uint8_t readErrorFlags();

    /**
     * @brief Check for RX buffer overflow flags in EFLG.
     * @param[out] rx0Overflow True if RX0 overflow occurred.
     * @param[out] rx1Overflow True if RX1 overflow occurred.
     * @return true if either overflow flag is set.
     */
    bool readRxOverflowFlags(bool& rx0Overflow, bool& rx1Overflow);

private:
    /**
     * @brief Optional convenience value storing a telemetry interval (ms).
     */
    uint16_t m_telemetryInterval;

    /**
     * @brief Internal receive buffer that stores up to 8 bytes from the last
     * received CAN frame.
     */
    uint8_t m_receivedData[UniversalPacker::CAN_MAX_DLC];

    /**
     * @brief Number of valid bytes currently stored in `m_receivedData`.
     */
    size_t m_receivedSize;

    /**
     * @brief CAN identifier for the last received frame (standard 11-bit ID).
     */
    uint32_t m_lastReceivedId = 0;

    /**
     * @brief SPI chip-select pin used when constructing this Telemetry object.
     */
    uint8_t m_mcpPin;
};

#endif
