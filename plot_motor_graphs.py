import pandas as pd
import matplotlib.pyplot as plt

# Example CSV filenames (replace with your actual files)
csv_files = ["joint_positions_dynamic_friction.csv", "joint_positions_no_friction.csv", "joint_positions_static_friction.csv", "joint_positions_viscous_friction.csv"]

plt.figure(figsize=(8, 6))

for file in csv_files:
    try:
        # Load CSV (no header assumed)
        df = pd.read_csv(file, header=None)
        first_col = df.iloc[:, 0]

        # Plot as dots
        plt.plot(first_col, 'o', markersize=3, label=file)

    except Exception as e:
        print(f"Error processing {file}: {e}")

plt.title("Comparison of First Column (Dot Graph)")
plt.xlabel("Index")
plt.ylabel("Value")
plt.legend()
plt.grid(True)
plt.show()
