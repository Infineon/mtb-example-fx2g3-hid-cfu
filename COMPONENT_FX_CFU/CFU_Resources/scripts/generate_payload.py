import os
import sys

def main(input_binary_file: str, output_binary_file: str):
    print("[CFU] Payload Binary Generation START")
    file_stats = os.path.getsize(input_binary_file)
    content_packet_size = 32
    num_whole_packets = int(file_stats/content_packet_size)
    last_pkt_bytes = file_stats % content_packet_size
    print(f"Input Binary File Size: {file_stats} Bytes")
    print(f"Number of 32Byte content packets: {num_whole_packets}")
    if last_pkt_bytes:
        print(f"Last content packet size: {last_pkt_bytes} Bytes")

    with open(input_binary_file, "rb") as input_binary:
        with open(output_binary_file, "wb") as output_binary:
            pkt_start_address = 0
            
            for pkt_counter in range(num_whole_packets):
                comp_bytes = input_binary.read(content_packet_size)
                output_binary.write(pkt_start_address.to_bytes(4, "little"))
                output_binary.write(content_packet_size.to_bytes(1, "little"))
                output_binary.write(comp_bytes)
                pkt_start_address += content_packet_size

            if last_pkt_bytes != 0:
                content_packet_size = last_pkt_bytes
                comp_bytes = input_binary.read (content_packet_size)
                output_binary.write(pkt_start_address.to_bytes(4, "little"))
                output_binary.write(content_packet_size.to_bytes(1, "little"))
                output_binary.write(comp_bytes)
            print ('Payload file created:', output_binary_file)
    print("[CFU] Payload Binary Generation END")


if __name__=="__main__":
    main(str(sys.argv[1]), str(sys.argv[2]))