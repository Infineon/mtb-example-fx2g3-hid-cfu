import subprocess
import sys

def main(input_binary_file: str):
    print()
    try:
        offer_process = subprocess.Popen(['python', 'scripts\\generate_offer.py', input_binary_file, '.'.join([input_binary_file[:-4], 'offer.bin'])])
    except:
        offer_process = subprocess.Popen(['python', 'scripts/generate_offer.py', input_binary_file, '.'.join([input_binary_file[:-4], 'offer.bin'])])

    offer_process.wait()
    print()

    try:
        payload_process = subprocess.Popen(['python', 'scripts\\generate_payload.py', input_binary_file, '.'.join([input_binary_file[:-4], 'payload.bin'])])
    except:
        payload_process = subprocess.Popen(['python', 'scripts/generate_payload.py', input_binary_file, '.'.join([input_binary_file[:-4], 'payload.bin'])])

    payload_process.wait()
    print()

if __name__=="__main__":
    main(str(sys.argv[1]))