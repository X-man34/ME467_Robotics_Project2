import requests
import pandas as pd
import time

# Define the IP and port of the device to request data from
server_ip = "http://192.168.12.149:80"

# Define the frequency of the data sampling (samples per second)
sampling_frequency = 100  # 1 Hz (you can adjust this value)

# Initialize an empty list to store the data
data = []


def send_command(command):
    try:
        clear_response = requests.get(f"{server_ip}/control?cmd={command}")
        clear_response.raise_for_status()  # Check for any errors in the response
        print("Command: " + str(command) + " sent successfully")
    except requests.exceptions.RequestException as e:
        print(f"Failed to send command: {e}")


send_command("clear")
send_command("start")
# Run for a specific duration or until a condition (e.g., max samples)
for _ in range(10):  # Change 10 to any number of iterations you want
    timestamp = time.time()

    # Request accelerometer data
    acc_response = requests.get(f"{server_ip}/get?accX=single&acc_time=single&accY=single&accZ=single")
    print(acc_response.content)
    print()

    # # Request gyroscope data
    # gyro_response = requests.get(f"{server_ip}/get?gyroX=full&gyro_time=full&gyroY=full&gyroZ=full")
    
    # # Request magnetometer data
    # mag_response = requests.get(f"{server_ip}/get?magX=full&mag_time=full&magY=full&magZ=full")
    
    # Extract the relevant data from the JSON responses
    # The keys will depend on the structure of the JSON response, adjust if necessary.
    # ax, ay, az = acc_data['accX'], acc_data['accY'], acc_data['accZ']
    # ωx, ωy, ωz = gyro_data['gyroX'], gyro_data['gyroY'], gyro_data['gyroZ']
    # mx, my, mz = mag_data['magX'], mag_data['magY'], mag_data['magZ']
    
    # # Prepare the row to append to the data list
    # data_row = [timestamp, mx, my, mz, ωx, ωy, ωz, ax, ay, az]
    # data.append(data_row)
    
    # Wait for the next sampling based on the specified frequency
    time.sleep(1 / sampling_frequency)
send_command("stop")

# Convert the collected data into a pandas DataFrame
df = pd.DataFrame(data, columns=["t", "mx", "my", "mz", "ωx", "ωy", "ωz", "ax", "ay", "az"])

# Print the DataFrame to verify
print(df)


