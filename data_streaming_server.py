import requests
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

# ==== CONFIG ====
server_ip = "http://192.168.12.149:80"
sampling_interval = 0.1  # 10 Hz
window_size_seconds = 5  # plot the last 5 seconds

# ==== INIT ====
acc_time_all = []
accX_all = []
accY_all = []
accZ_all = []


fig, axs = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

end_of_last_sample = 0
num_iterations = 0
# ==== FETCHING ====
def fetch_data():
    global end_of_last_sample, num_iterations
    try:

        response = requests.get(f"{server_ip}/get?accX=full&acc_time=full&accY=full&accZ=full")
        num_iterations += 1
        response.raise_for_status()
        json_data = response.json()
        with open("responses.txt", "a") as file:
            file.write(str(json_data) + "\n\n")
        # Extract the sample lists from the JSON data.
        # The JSON format is expected to be:
        # {
        #   "buffer": {
        #       "acc_time": {"buffer": [...], ...},
        #       "accX": {"buffer": [...], ...},
        #       "accY": {"buffer": [...], ...},
        #       "accZ": {"buffer": [...], ...}
        #   },
        #   "status": { ... }
        # }
        buffers = json_data.get("buffer", {})
        time_samples = buffers.get("acc_time", {}).get("buffer", [])
        x_samples    = buffers.get("accX", {}).get("buffer", [])
        y_samples    = buffers.get("accY", {}).get("buffer", [])
        z_samples    = buffers.get("accZ", {}).get("buffer", [])
        
        min_len = min(len(time_samples), len(x_samples), len(y_samples), len(z_samples))
        if min_len > 0:
            end_of_last_sample += min_len - end_of_last_sample
            print(f"Iteration {num_iterations+1}/{num_iterations} appended {min_len - end_of_last_sample} samples.")
            return time_samples[end_of_last_sample:min_len], x_samples[end_of_last_sample:min_len], y_samples[end_of_last_sample:min_len], z_samples[end_of_last_sample:min_len]

        else:
            print(f"Warning: No valid samples at iteration {num_iterations+1}")
            return [], [], [], []
    except Exception as e:
        print(f"Fetch error: {e}")
        return [], [], [], []


# ==== UPDATE FUNCTION ====
def update(frame):
    global acc_time_all, accX_all, accY_all, accZ_all

    t, x, y, z = fetch_data()
    if not t:
        return

    acc_time_all.extend(t)
    accX_all.extend(x)
    accY_all.extend(y)
    accZ_all.extend(z)

    # Trim to window size
    if len(acc_time_all) > 0:
        current_time = acc_time_all[-1]
        min_time = current_time - window_size_seconds
        mask = [i for i, ts in enumerate(acc_time_all) if ts >= min_time]
        acc_time_all = [acc_time_all[i] for i in mask]
        accX_all = [accX_all[i] for i in mask]
        accY_all = [accY_all[i] for i in mask]
        accZ_all = [accZ_all[i] for i in mask]

        # Clear and redraw plots
        axs[0].cla()
        axs[1].cla()
        axs[2].cla()

        axs[0].plot(acc_time_all, accX_all, color='tab:blue')
        axs[0].set_ylabel("Acc X")
        axs[0].grid(True)

        axs[1].plot(acc_time_all, accY_all, color='tab:orange')
        axs[1].set_ylabel("Acc Y")
        axs[1].grid(True)

        axs[2].plot(acc_time_all, accZ_all, color='tab:green')
        axs[2].set_ylabel("Acc Z")
        axs[2].set_xlabel("Time (s)")
        axs[2].grid(True)

        fig.tight_layout()

def send_command(command):
    try:
        clear_response = requests.get(f"{server_ip}/control?cmd={command}")
        clear_response.raise_for_status()  # Check for any errors in the response
        print("Command: " + str(command) + " sent successfully")
    except requests.exceptions.RequestException as e:
        print(f"Failed to send command: {e}")



def traditional_plotting():
    send_command("clear")
    send_command("start")
    # Run for a specific duration or until a condition (e.g., max samples)
    for i in range(num_iterations):  # Change 10 to any number of iterations you want

        try:
            response = requests.get(f"{server_ip}/get?accX=full&acc_time=full&accY=full&accZ=full")
            response.raise_for_status()
            json_data = response.json()
            with open("responses.txt", "a") as file:
                file.write(str(json_data) + "\n\n")
            # Extract the sample lists from the JSON data.
            # The JSON format is expected to be:
            # {
            #   "buffer": {
            #       "acc_time": {"buffer": [...], ...},
            #       "accX": {"buffer": [...], ...},
            #       "accY": {"buffer": [...], ...},
            #       "accZ": {"buffer": [...], ...}
            #   },
            #   "status": { ... }
            # }
            buffers = json_data.get("buffer", {})
            time_samples = buffers.get("acc_time", {}).get("buffer", [])
            x_samples    = buffers.get("accX", {}).get("buffer", [])
            y_samples    = buffers.get("accY", {}).get("buffer", [])
            z_samples    = buffers.get("accZ", {}).get("buffer", [])
            
            min_len = min(len(time_samples), len(x_samples), len(y_samples), len(z_samples))
            if min_len > 0:
                acc_time_all.extend(time_samples[end_of_last_sample:min_len])
                accX_all.extend(x_samples[end_of_last_sample:min_len])
                accY_all.extend(y_samples[end_of_last_sample:min_len])
                accZ_all.extend(z_samples[end_of_last_sample:min_len])
                end_of_last_sample += min_len - end_of_last_sample
                print(f"Iteration {i+1}/{num_iterations} appended {min_len - end_of_last_sample} samples.")
            else:
                print(f"Warning: No valid samples at iteration {i+1}")
                
        except requests.exceptions.RequestException as e:
            print("Request error:", e)
        except Exception as ex:
            print("An error occurred processing data:", ex)

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


    # Convert the collected data into a pandas DataFrame.
    # Each row is one sample point.
    df = pd.DataFrame({
        "time": acc_time_all,
        "accX": accX_all,
        "accY": accY_all,
        "accZ": accZ_all
    })

    # Print a brief summary of the DataFrame and the first few rows
    print("Collected Data:")
    print(df.head())
    print(df.describe())
    df.to_excel('filename.xlsx', index=False)

    # Create three subplots for each acceleration component
    fig, axs = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

    # Plot Acceleration X
    axs[0].plot(df["time"], df["accX"], color="tab:blue")
    axs[0].set_ylabel("Acc X")
    axs[0].set_title("Acceleration X vs Time")
    axs[0].grid(True)

    # Plot Acceleration Y
    axs[1].plot(df["time"], df["accY"], color="tab:orange")
    axs[1].set_ylabel("Acc Y")
    axs[1].set_title("Acceleration Y vs Time")
    axs[1].grid(True)

    # Plot Acceleration Z
    axs[2].plot(df["time"], df["accZ"], color="tab:green")
    axs[2].set_ylabel("Acc Z")
    axs[2].set_title("Acceleration Z vs Time")
    axs[2].set_xlabel("Time (s)")
    axs[2].grid(True)

    plt.tight_layout()
    plt.show()

send_command("clear")
send_command("start")
print("Starting real-time plot...")
ani = FuncAnimation(fig, update, interval=sampling_interval * 1000)
plt.show()