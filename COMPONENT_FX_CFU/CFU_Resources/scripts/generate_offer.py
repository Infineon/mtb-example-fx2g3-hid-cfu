#---------------------------------------------------------------
# CFU Offer file format 
#---------------------------------------------------------------
#	Byte(s) |  Value
#---------------------------------------------------------------
#	15:14   |  (PI)  Product ID is 2 bytes		0x0000 to 0xFFFF
#	13      |  (R1)  Reserved1 5-bit register
#			|  (MS)  Milestone 3-bit register	0x0 to 0x7
#	12      |  (R2)  Reserved2 2-bit register
#			|  (BK)  Bank 2-bit register	0x0 - 0x3
#			|  (PR)  Protocol Revision  4-bit register	(0x0 TO 0xF)
#	11:8    |  (VM)  Hardware Variant Mask 32-bit register
#	7:4     |  (VN)  Version 32-bit register 			(Extracted from BIN FILE 0x00000000 to 0xFFFFFFFF)
#	3       |  (TK)  Token byte 8-bit register			(Valid values 0-255)
#	2       |  (CI)  Component ID 8-bit register		(Valid values 0-255)
#	1       |  (FV)  Force version bit  1-bit register 	(Valid Values 0 or 1 -> For any other value last bitwill be used and rest discarded)
#			|  (FR)  Force Immediate Reset  1-bit register (Valid Values 0 or 1 -> For any other value last bitwill be used and rest discarded)
#			|  (R0)  Reserved0 6-bit register
#	0       |  (SN)  Segment Number 8-bit register (Valid values 0-255)

import os
import sys

def get_version(bytes):
    version = []
    literal = []
    index = 0
    for byte in bytes:
        if byte == '0x2e':
            v = "".join(literal)
            literal = []
            version.append(v)
            index += 1
        elif byte == '0xa' or byte == '0x0a':
            v = "".join(literal)
            literal = []
            version.append(v)
            break
        else:
            literal.append(chr(int(byte[2:], 16)))
    return version

def get_bytes(v_hex):
    bytes = []
    flag = False
    for byte in v_hex:
        if byte == ' ':
            bytes.append(hex(buffer))
        elif not flag:
            buffer = 0x0
            buffer = int(byte, 16) << 0b0100
            flag = True
        elif flag:
            buffer+=int(byte, 16)
            flag = False

    return bytes

def fileOneByteChecksum(binary_file: str):
    checksum = 0
    with open(binary_file, 'rb') as binary:
        while True:
            byte = binary.read(1)
            if not byte:
                break
            checksum += int.from_bytes(byte)
    return checksum


def main(input_binary_file: str, output_binary_file: str):
    print("[CFU] Offer Binary Generation START")

    # 1. Open the bin file, extract data from words of bytes [14, 30)
    bin_file = input_binary_file
    bytes = []
    v_hex = ""
    # CFU Protocol document
    #
    # 5.2.1.3 Vendor Specific: These four bytes may be used to encode any custom information
    # in the offer that is specific to vendor implementation.
    #
    # Currently, we use these bytes to indicate 1Byte checksum of the upcoming firmware.
    hwVariantMask = fileOneByteChecksum(input_binary_file)
    with open(bin_file, 'rb') as binary:
        version = binary.read(36)    # Maximum extent of human readable version info from the binary
        v_hex = version[14:36].hex(' ')
    
    # 2. Get bytes from extracted hex words
    bytes = get_bytes(v_hex)

    # 3. Extract version from bytes
    version_list = get_version(bytes)
    raw_version = 0
    for v_section in version_list:
        raw_version = raw_version << 0b1000
        raw_version += int(v_section)
    
    # 4. Write bytes to bin file
    # Byte 1
    seg_no = 0b00                   # Used if firmware image is broken down into smaller segments
    byte1 = seg_no

    # Byte 2
    forceIgnoreVersion = 0b1        # Request device to ignore incoming offer version information
    forceReset = 0b0                # Request device to reset after CFU completion
    reserved6b = 0b000000
    byte2 = (forceIgnoreVersion << 7) + (forceReset << 6) + (reserved6b)

    # Byte 3
    componentId = 0x00              # Can be used for selection amongst multiple components that may be updated
    byte3 = componentId

    # Byte 4
    token = 115                     # A unique token, when device responds with the same token, we know its responding to our CFU host tool instance
    byte4 = token

    # Byte 5, 6, 7, 8
    byte5_6_7_8 = raw_version       # Extracted from the input binary file

    # Bytes 9, 10, 11, 12
    byte9_10_11_12 = hwVariantMask  # Vendor-specific implementation, used here to indicate 1Byte checksum of the upcoming firmware.

    # Byte 13
    reserved2b = 0b00
    bank = 0b00                     # Used if update is selectively made amongst different firmware banks
    protocolRevision = 0b0010       # Indicate CFU protocol version
    byte13 = (reserved2b << 6) + (bank << 4) + protocolRevision

    # Byte 14
    reserved5b = 0b00000
    milestone = 0b110               # Vendor-defined field
    byte14 = (reserved5b << 3) + milestone

    # Byte 15, 16
    productId = 0xF203
    byte15_16 = productId           # Vendor-defined product identification

    with open(output_binary_file, "wb") as binary:
        binary.write(byte1.to_bytes(1, "little"))
        binary.write(byte2.to_bytes(1, "little"))
        binary.write(byte3.to_bytes(1, "little"))
        binary.write(byte4.to_bytes(1, "little"))
        binary.write(byte5_6_7_8.to_bytes(4, "little"))
        binary.write(byte9_10_11_12.to_bytes(4, "little"))
        binary.write(byte13.to_bytes(1, "little"))
        binary.write(byte14.to_bytes(1, "little"))
        binary.write(byte15_16.to_bytes(2, "little"))

    print(f"segmentNumber: {byte1}")
    print(f"forceIgnoreVersion: {(byte2 & 0b10000000) >> 7}")
    print(f"forceReset: {(byte2 & 0b01000000)>> 6}")
    print(f"componentId: {byte3}")
    print(f"token: {byte4}")
    print(f"version: {'.'.join(version_list)}")
    print(f"hwVariantMask: {hex(byte9_10_11_12)}")
    print(f"bank: {(byte13 & 0b00110000) >> 4}")
    print(f"protocolRevision: {byte13 & 0b1111}")
    print(f"milestone: {byte14 & 0b111}")
    print(f"productId: {byte15_16}")
    print(f"Offer file created: {output_binary_file}")
    print("[CFU] Offer Binary Generation END")

if __name__=="__main__":
    main(str(sys.argv[1]), str(sys.argv[2]))