import spatialmath as sm
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from Question2 import *
from filters import *
from pathlib import Path


if __name__ == "__main__":
    #Question 4 part b, the triad method, is implemented in the filters file. 


    data_path = Path("csv_files") / "question2_input.csv"
    csv_data = pd.read_csv(str(data_path), header=None,
                    names=['t', 'mx', 'my', 'mz', 'gyrox', 'gyroy', 'gyroz', 'ax', 'ay', 'az'])
    dt = 0.01

    times = []
    rotation_angles = []
    triad_estimator = TriadEstimator(dt, kP=2)
    # Perfrom the simulation and get the data
    times, rotation_angles, _, _, _, _, _ = simulate_and_visualize_data(csv_data, dt, triad_estimator,  do_3D_vis=True, show_body_coords=False, show_extra_vectors=True, show_spatial_coords=True)

    #Plot the data. 
    plot_rotation_data(times, rotation_angles, title_suffix=" using Triad Orientation Estimator", data_in_radians=True, convert=False)
