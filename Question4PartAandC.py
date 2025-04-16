import spatialmath as sm
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from Question2 import plot_rotation_data
from filters import NaiveEstimator
from visualizer import simulate_and_visualize_data

if __name__ == "__main__":
    # Load the CSV file
    data_path = Path("csv_files") / "question2_input.csv"
    csv_data = pd.read_csv(str(data_path), header=None,
                    names=['t', 'mx', 'my', 'mz', 'gyrox', 'gyroy', 'gyroz', 'ax', 'ay', 'az'])
    
    dt = 0.01

    times = []
    rotation_angles = []

    naive_estimator = NaiveEstimator(dt)
    # Perfrom the simulation and get the data
    times, rotation_angles, _, _, _, _, _ = simulate_and_visualize_data(csv_data, dt, naive_estimator,  do_3D_vis=True, show_body_coords=False, show_extra_vectors=False, show_spatial_coords=True)

    #Plot the data. 
    plot_rotation_data(times, rotation_angles, title_suffix=" using Naive Orientation Estimator", data_in_radians=True, convert=False)
