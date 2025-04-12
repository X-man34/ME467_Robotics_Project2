import requests, time, threading, bisect, traceback, math, json
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import numpy as np


server_ip = "http://192.168.12.149:80"
sampling_interval = 0.01  # 100 Hz
data_queue = []

def data_queue_updater(collectionTime=3, fetch_interval=0.125):
    """
    This function runs in a separate thread and continuously fetches data from the phone, putting it into the data queue.
    It will run for a specified amount of time (collectionTime) and then stop.
    The data is fetched from the server using a GET request, and the response is expected to be in JSON format.

    Args:
        collectionTime (int): The duration in seconds for which to collect data.
        fetch_interval (float): The interval in seconds between each data fetch.
    """
    global data_queue, server_ip
    send_command("clear")
    send_command("start")
    start_time = time.time()
    end_time = start_time + collectionTime
    current_index = 0
    while time.time() < end_time:
        try:
            fetch_start = time.time()
            # Fetch data from the server

            

            # Your last known timestamp from gyro_time
            last_time = data_queue[-1][0] if data_queue else 0
            print(f"Last time: {last_time}")
            # Construct the partial GET request
            if last_time != 0:
                partial_url = (
                    f"{server_ip}/get?"
                    f"accX={last_time}|gyro_time&accY={last_time}|gyro_time&accZ={last_time}|gyro_time&"
                    f"gyroX={last_time}|gyro_time&gyroY={last_time}|gyro_time&gyroZ={last_time}|gyro_time&"
                    f"magX={last_time}|gyro_time&magY={last_time}|gyro_time&magZ={last_time}|gyro_time&"
                    f"gyro_time={last_time}")
                response = requests.get(partial_url)
                response.raise_for_status()
                partial_data = response.json()

                # Extract the sample lists from the JSON data.
                buffer = partial_data.get("buffer", {})
                times = buffer.get("gyro_time", {}).get("buffer", [])
                accel_vectors = list(zip(buffer.get("accX", {}).get("buffer", []),
                                            buffer.get("accY", {}).get("buffer", []),
                                            buffer.get("accZ", {}).get("buffer", [])))
                gyro_vectors = list(zip(buffer.get("gyroX", {}).get("buffer", []),
                                buffer.get("gyroY", {}).get("buffer", []),
                                buffer.get("gyroZ", {}).get("buffer", [])))
                mag_vectors = list(zip(buffer.get("magX", {}).get("buffer", []),
                                buffer.get("magY", {}).get("buffer", []),
                                buffer.get("magZ", {}).get("buffer", [])))
                data_queue.extend([[t, av, gv, mv] for t, av, gv, mv in zip(times, accel_vectors, gyro_vectors, mag_vectors)])

                # with open("responses.json", "a") as file:
                #     file.write("Begin fetch:\n Old method\nAcceleration data:\n")
                #     json.dump(accel_data_response.json(), file, indent=4)
                #     file.write("\nGyroscope data:\n")
                #     json.dump(gyro_data_response.json(), file, indent=4)
                #     file.write("\nMagnetometer data:\n")
                #     json.dump(mag_data_response.json(), file, indent=4)
                #     file.write(f"\nPartial data, last time{last_time}:\n")
                #     json.dump(partial_data, file, indent=4)
                #     file.write("\nEnd Fetch\n")
            else:
                accel_data_response = requests.get(f"{server_ip}/get?accX=full&acc_time=full&accY=full&accZ=full")
                accel_data_response.raise_for_status()  # Check for any errors in the response
                accel_data = accel_data_response.json()

                gyro_data_response = requests.get(f"{server_ip}/get?gyroX=full&gyro_time=full&gyroY=full&gyroZ=full")
                gyro_data_response.raise_for_status()
                gyro_data = gyro_data_response.json()

                mag_data_response = requests.get(f"{server_ip}/get?magX=full&mag_time=full&magY=full&magZ=full")
                mag_data_response.raise_for_status()
                mag_data = mag_data_response.json()


                # Extract the sample lists from the JSON data.

                # Acceleration
                accel_buffers = accel_data.get("buffer", {})
                accel_times = accel_buffers.get("acc_time", {}).get("buffer", [])
                accel_vectors = list(zip(accel_buffers.get("accX", {}).get("buffer", []),
                                        accel_buffers.get("accY", {}).get("buffer", []),
                                        accel_buffers.get("accZ", {}).get("buffer", [])))

                # Gyroscope
                gyro_buffers = gyro_data.get("buffer", {})
                times = gyro_buffers.get("gyro_time", {}).get("buffer", [])
                gyro_vectors = list(zip(gyro_buffers.get("gyroX", {}).get("buffer", []),
                                        gyro_buffers.get("gyroY", {}).get("buffer", []),
                                        gyro_buffers.get("gyroZ", {}).get("buffer", [])))

                # Magnetometer
                mag_buffers = mag_data.get("buffer", {})
                mag_times = mag_buffers.get("mag_time", {}).get("buffer", [])
                mag_vectors = list(zip(mag_buffers.get("magX", {}).get("buffer", []),
                                        mag_buffers.get("magY", {}).get("buffer", []),
                                        mag_buffers.get("magZ", {}).get("buffer", [])))
                
                
                #Since we are hijacking this open source code, each response give the full history of the data, (I know it gets expontially less efficient, but lets just not use it for long amounts of time...)
                # Becuase of this we need to trim off the data we already have. 
                min_len = min(len(accel_times), len(gyro_vectors), len(mag_vectors), len(times), len(accel_vectors))
                accel_times = accel_times[:min_len]
                accel_vectors = accel_vectors[:min_len]
                gyro_vectors = gyro_vectors[:min_len]
                mag_vectors = mag_vectors[:min_len]
                times = times[:min_len]
                if len(times) != 0:
                    insert_pos = bisect.bisect_left(accel_times, times[0])

                    # Handle cases where insert_pos is at the boundaries of the list
                    if insert_pos == 0:
                        closest_index = 0
                    elif insert_pos == len(accel_times):
                        closest_index = len(accel_times) - 1
                    else:
                        # Compare the element before and after the insertion position
                        before = accel_times[insert_pos - 1]
                        after = accel_times[insert_pos]
                        if abs(before - times[0]) <= abs(after - times[0]):
                            closest_index = insert_pos - 1
                        else:
                            closest_index = insert_pos

                    accel_times = accel_times[closest_index:]
                    accel_vectors = accel_vectors[closest_index:]
                    # we will also need to correlate the acceleration data to the gyro and mag data, although they accel times do not exactly match up. 
                    # To do this we will find the closest time in the accel array to the first gyro time and trim off all preceding accel data. 
                    print("Ready to add times to the data queue")
                    data_queue.extend([[t, av, gv, mv] for t, av, gv, mv in zip(times, accel_vectors, gyro_vectors, mag_vectors)])
                    print("Quee size after adding: " + str(len(data_queue)))

                current_index += len(times)
            fetch_end = time.time()
            fetch_time = fetch_end - fetch_start
            print(f"Fetched {len(times)} samples. in {fetch_time:.2f} seconds")
        except Exception as e:
            print(f"Fetch error: {e}")
            traceback.print_exc()
        time.sleep(fetch_interval)
    send_command("stop")



def send_command(command):
    try:
        clear_response = requests.get(f"{server_ip}/control?cmd={command}")
        clear_response.raise_for_status()  # Check for any errors in the response
        print("Command: " + str(command) + " sent successfully")
    except requests.exceptions.RequestException as e:
        print(f"Failed to send command: {e}")




# Start collecting data and storing it in the buffer. 
data_updater_thread = threading.Thread(target=data_queue_updater, args=(90, 0.125))
data_updater_thread.start()
time.sleep(.25)# Let the buffer build up a bit so we dont' run out of data to graph. 
graphing_start_time = time.time() # This is what we will call time 0 for the purposes of the graph, and "corresponds" to time 0 in the data, even though its later. 



import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Variables to control which plots show up (set to True if you want the plot to show)
plot_acc = True
plot_gyro = True
plot_mag = True
plot_acc_x = True
plot_acc_y = True
plot_acc_z = True
plot_gyro_x = True
plot_gyro_y = True
plot_gyro_z = True
plot_mag_x = True
plot_mag_y = True
plot_mag_z = True

n_seconds = 5  # Last N seconds to plot

# Create a figure for each sensor type (acceleration, gyro, magnetometer)
fig_acc = plt.figure("Acceleration Components", figsize=(10, 8)) if plot_acc else None
fig_gyro = plt.figure("Gyroscope Components", figsize=(10, 8)) if plot_gyro else None
fig_mag = plt.figure("Magnetometer Components", figsize=(10, 8)) if plot_mag else None

# Acceleration subplots (X, Y, Z)
ax1_acc = fig_acc.add_subplot(311, label="AccX") if plot_acc_x else None
ax2_acc = fig_acc.add_subplot(312, label="AccY") if plot_acc_y else None
ax3_acc = fig_acc.add_subplot(313, label="AccZ") if plot_acc_z else None

# Gyroscope subplots (X, Y, Z)
ax1_gyro = fig_gyro.add_subplot(311, label="GyroX") if plot_gyro_x else None
ax2_gyro = fig_gyro.add_subplot(312, label="GyroY") if plot_gyro_y else None
ax3_gyro = fig_gyro.add_subplot(313, label="GyroZ") if plot_gyro_z else None

# Magnetometer subplots (X, Y, Z)
ax1_mag = fig_mag.add_subplot(311, label="MagX") if plot_mag_x else None
ax2_mag = fig_mag.add_subplot(312, label="MagY") if plot_mag_y else None
ax3_mag = fig_mag.add_subplot(313, label="MagZ") if plot_mag_z else None

# Set up limits, labels, and grid for each axis
for ax in [ax1_acc, ax2_acc, ax3_acc]:
    if ax:
        ax.set_ylim(-50, 50)
        ax.grid(True)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Acceleration (m/s²)")

for ax in [ax1_gyro, ax2_gyro, ax3_gyro]:
    if ax:
        ax.set_ylim(-50, 50)
        ax.grid(True)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Gyroscope (rad/s)")

for ax in [ax1_mag, ax2_mag, ax3_mag]:
    if ax:
        ax.set_ylim(-50, 50)
        ax.grid(True)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Magnetometer (µT)")

def update(frame):
    global graphing_start_time
    if not data_queue:
        return

    current_time = data_queue[-1][0]
    cutoff = current_time - n_seconds

    relevant_data = [item for item in data_queue if item[0] >= cutoff]
    times = [t - graphing_start_time for t, _, _, _ in relevant_data]
    
    # Acceleration components
    acc_x = [a[0] for _, a, _, _ in relevant_data]
    acc_y = [a[1] for _, a, _, _ in relevant_data]
    acc_z = [a[2] for _, a, _, _ in relevant_data]

    # Gyroscope components
    gyro_x = [g[0] for _, _, g, _ in relevant_data]
    gyro_y = [g[1] for _, _, g, _ in relevant_data]
    gyro_z = [g[2] for _, _, g, _ in relevant_data]

    # Magnetometer components
    mag_x = [m[0] for _, _, _, m in relevant_data]
    mag_y = [m[1] for _, _, _, m in relevant_data]
    mag_z = [m[2] for _, _, _, m in relevant_data]

    # Update data for each subplot, based on visibility flags
    if ax1_acc and plot_acc_x:
        ax1_acc.plot(times, acc_x, color='red')
    if ax2_acc and plot_acc_y:
        ax2_acc.plot(times, acc_y, color='green')
    if ax3_acc and plot_acc_z:
        ax3_acc.plot(times, acc_z, color='blue')

    if ax1_gyro and plot_gyro_x:
        ax1_gyro.plot(times, gyro_x, color='red')
    if ax2_gyro and plot_gyro_y:
        ax2_gyro.plot(times, gyro_y, color='green')
    if ax3_gyro and plot_gyro_z:
        ax3_gyro.plot(times, gyro_z, color='blue')

    if ax1_mag and plot_mag_x:
        ax1_mag.plot(times, mag_x, color='red')
    if ax2_mag and plot_mag_y:
        ax2_mag.plot(times, mag_y, color='green')
    if ax3_mag and plot_mag_z:
        ax3_mag.plot(times, mag_z, color='blue')

    return

ani_acc = FuncAnimation(fig_acc, update, interval=10) if fig_acc else None
ani_gyro = FuncAnimation(fig_gyro, update, interval=10) if fig_gyro else None
ani_mag = FuncAnimation(fig_mag, update, interval=10) if fig_mag else None

plt.tight_layout()
plt.show()
