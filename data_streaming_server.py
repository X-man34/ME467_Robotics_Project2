import requests, time, threading, bisect, traceback, math, json
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import numpy as np


server_ip = "http://192.168.12.149:80"
sampling_interval = 0.01  # 100 Hz
data_queue = deque()

def data_queue_updater(collectionTime=3, fetch_interval=0.25):
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
        
            # print length of the data queue'
            print(f"Data queue length: {len(data_queue)}")
            if len(data_queue) > 0:
                print(f"Last element in data queue: {data_queue[-1]}")
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
                
    


                if last_time == 0:
                    print("No last time, so we are getting the full data set")
                    print(f"Fetched {len(times)} time samples.")
                    print(f"Fetched {len(accel_vectors)} accel samples.")
                    print(f"Fetched {len(gyro_vectors)} gyro samples.")
                    print(f"Fetched {len(mag_vectors)} mag samples.")
                
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
data_updater_thread = threading.Thread(target=data_queue_updater, args=(90, 0.25))
data_updater_thread.start()
time.sleep(.25)# Let the buffer build up a bit so we dont' run out of data to graph. 
graphing_start_time = time.time() # This is what we will call time 0 for the purposes of the graph, and "corresponds" to time 0 in the data, even though its later. 


# Setup figure with 3 subplots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
fig.suptitle("Real-time Sensor Vector Magnitudes (Full History)")

# Data containers (grow as new data comes in)
time_data = []
acc_mag_data = []
gyro_mag_data = []
mag_mag_data = []

# Plot lines
line1, = ax1.plot([], [], label="Acceleration Magnitude", color="red")
line2, = ax2.plot([], [], label="Gyroscope Magnitude", color="green")
line3, = ax3.plot([], [], label="Magnetometer Magnitude", color="blue")

for ax in (ax1, ax2, ax3):
    ax.set_ylim(0, 50)  # Adjust based on expected range
    ax.legend()
    ax.grid(True)

ax1.set_ylabel("Accel (m/s²)")
ax2.set_ylabel("Gyro (rad/s)")
ax3.set_ylabel("Mag (µT)")
ax3.set_xlabel("Time (s)")

start_time = None

def vector_magnitude(vec):
    return math.sqrt(vec[0]**2 + vec[1]**2 + vec[2]**2)

def set_dynamic_ylim(ax, data, margin=0.1):
    if not data:
        return
    min_val = min(data)
    max_val = max(data)
    if min_val == max_val:
        # Add a small buffer if all values are the same
        ax.set_ylim(min_val - 1, max_val + 1)
    else:
        padding = (max_val - min_val) * margin
        ax.set_ylim(min_val - padding, max_val + padding)

def update(frame):
    global start_time

    if not data_queue:
        return line1, line2, line3

    while data_queue:
        t, acc, gyro, mag = data_queue.popleft()
        if start_time is None:
            start_time = t
        t -= start_time

        acc_mag_data.append(vector_magnitude(acc))
        gyro_mag_data.append(vector_magnitude(gyro))
        mag_mag_data.append(vector_magnitude(mag))
        time_data.append(t)

    # Update plot data
    line1.set_data(time_data, acc_mag_data)
    line2.set_data(time_data, gyro_mag_data)
    line3.set_data(time_data, mag_mag_data)

    # Update x-limits
    if time_data:
        for ax in (ax1, ax2, ax3):
            ax.set_xlim(0, time_data[-1])

    # Dynamically adjust y-limits
    set_dynamic_ylim(ax1, acc_mag_data)
    set_dynamic_ylim(ax2, gyro_mag_data)
    set_dynamic_ylim(ax3, mag_mag_data)

    return line1, line2, line3

ani = FuncAnimation(fig, update, interval=10)  # Call update every 100 ms
plt.tight_layout()
plt.show()