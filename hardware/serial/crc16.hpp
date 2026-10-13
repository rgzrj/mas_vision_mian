#ifndef _SERIAL_CRC16_H_
#define _SERIAL_CRC16_H_

#include <array>
#include <cstddef>
#include <cstdint>

namespace serial_crc16
{
namespace detail
{
constexpr std::array<uint16_t, 256> buildTable()
{
    std::array<uint16_t, 256> table{};
    for (uint16_t i = 0; i < 256; ++i)
    {
        uint16_t crc = i;
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0x8408) : static_cast<uint16_t>(crc >> 1);
        table[i] = crc;
    }
    return table;
}

inline constexpr auto TABLE = buildTable();
} // namespace detail

inline uint16_t checksum(const uint8_t *data, size_t size)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < size; ++i)
        crc = static_cast<uint16_t>((crc >> 8) ^ detail::TABLE[(crc ^ data[i]) & 0xFF]);
    return crc;
}
} // namespace serial_crc16

#endif // _SERIAL_CRC16_H_
