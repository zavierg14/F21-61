import time
import csv
import can

# -----------------------------
#  CAN bus setup
# -----------------------------
bustype = 'socketcan'
channel = 'can0'
bitrate = 500000
bus = can.interface.Bus(channel=channel, bustype=bustype, bitrate=bitrate)

# -----------------------------
#  Main receiving loop
# -----------------------------
def main():
    # Open CSV file once; log new rows indefinitely
    with open("potentiometer_received_data.csv", "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        # Write a header row
        writer.writerow(["Timestamp (s)", "Potentiometer Raw Value"])

        print("Listening on CAN bus, press Ctrl+C to exit.")

        try:
            while True:
                # Block until a message is received
                msg = bus.recv()
                if msg is None or len(msg.data) < 8:
                    continue  # Skip if no message or message is not 8 bytes

                # First 4 bytes: bits 0?19 = microseconds, bits 20?25 = seconds, bits 26?31 = minutes
                time_val = int.from_bytes(msg.data[0:4], byteorder='big', signed=False)
                microseconds = time_val & 0xFFFFF         # lower 20 bits
                seconds = (time_val >> 20) & 0x3F         # next 6 bits
                minutes = (time_val >> 26) & 0x3F         # next 6 bits

                # Convert this to a floating-point total (minutes*60 + seconds + microseconds/1e6)
                decoded_time = minutes * 60 + seconds + (microseconds / 1_000_000.0)

                # Next 4 bytes: raw ADC value
                pot_raw_value = int.from_bytes(msg.data[4:8], byteorder='big', signed=False)

                # Write a row to the CSV
                writer.writerow([f"{decoded_time:.6f}", pot_raw_value])

                # Optional debug print to console
                print(
                    f"Received CAN ID=0x{msg.arbitration_id:X}, "
                    f"Decoded Time={decoded_time:.6f}s, "
                    f"Raw Value={pot_raw_value}"
                )

        except KeyboardInterrupt:
            print("\nExiting receive script.")

if __name__ == "__main__":
    main()
