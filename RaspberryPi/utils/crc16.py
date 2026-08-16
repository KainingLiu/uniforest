"""
CRC16-CCITT (polynomial 0x1021, initial 0xFFFF, no reflect, no xor-out).
Matches the STM32 implementation in protocol.c.
"""


def crc16_ccitt(data: bytes) -> int:
    """Compute CRC16-CCITT over a byte string."""
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
            crc &= 0xFFFF  # keep 16-bit
    return crc
