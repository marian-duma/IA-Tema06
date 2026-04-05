import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os

WORK_DIR = os.path.dirname(os.path.abspath(__file__))

input_path =  WORK_DIR + '/tema_b_log_braitenberg.csv'

df = pd.read_csv(input_path)

# Create a figure with 3 subplots
fig, axes = plt.subplots(3, 1, figsize=(10, 15))
plt.subplots_adjust(hspace=0.4)

# --- Graph 1: XY Trajectory ---
axes[0].plot(df['pos_x'], df['pos_y'], label='Robot Path', color='blue')
axes[0].set_title('Robot Trajectory (XY Plan)')
axes[0].set_xlabel('Position X (m)')
axes[0].set_ylabel('Position Y (m)')
axes[0].grid(True)
axes[0].axis('equal')

# --- Graph 2: Velocities vs Time ---
axes[1].plot(df['timestamp'], df['v_left'], label='v_left', alpha=0.8)
axes[1].plot(df['timestamp'], df['v_right'], label='v_right', alpha=0.8)
axes[1].set_title('Wheel Velocities over Time')
axes[1].set_xlabel('Time (s)')
axes[1].set_ylabel('Velocity (rad/s)')
axes[1].legend()
axes[1].grid(True)

# --- Graph 3: Sensor Heatmap ---
sensor_cols = [f's{i}' for i in range(8)]
sensor_data = df[sensor_cols].T 

sns.heatmap(sensor_data, ax=axes[2], cmap='YlOrRd_r', 
            xticklabels=20, yticklabels=sensor_cols)
axes[2].set_title('Sensor Activations (s0-s7)')
axes[2].set_xlabel('Iteration / Time')
axes[2].set_ylabel('Sensor Index')

output_path = WORK_DIR + '/grafice_braitenberg.png'
plt.savefig(output_path, dpi=300, bbox_inches='tight')
print(f"Graficele au fost salvate în: {output_path}")
plt.show()
