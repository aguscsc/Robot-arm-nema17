import numpy as np
import matplotlib.pyplot as plt


def generate_workspace(L0, L1, L2):
    print(f"Calculating workspace for L0={L0}m, L1={L1}m, L2={L2}m...")

    # 1. Define Physical Joint Limits (in radians)
    theta1 = np.linspace(-np.pi, np.pi, 60)  # Base: Full 360 rotation
    theta2 = np.linspace(0, np.pi, 50)  # Shoulder: 0 to 180 degrees
    theta3 = np.linspace(
        -np.pi + 0.1, -0.1, 50
    )  # Elbow: Prevents bending backwards into L1

    # 2. Create a 3D grid of all angle combinations
    T1, T2, T3 = np.meshgrid(theta1, theta2, theta3)

    # 3. Vectorized Forward Kinematics
    # Calculate the horizontal radius (r) from the center pedestal
    r = L1 * np.cos(T2) + L2 * np.cos(T2 + T3)

    # Convert cylindrical (r, theta1, z) to Cartesian (X, Y, Z)
    X = r * np.cos(T1)
    Y = r * np.sin(T1)
    Z = L0 + L1 * np.sin(T2) + L2 * np.sin(T2 + T3)

    # 4. Flatten the matrices into 1D arrays for plotting
    X_flat = X.flatten()
    Y_flat = Y.flatten()
    Z_flat = Z.flatten()

    # Optional: Filter out points that clip through the floor (Z < 0)
    floor_mask = Z_flat >= 0
    X_valid = X_flat[floor_mask]
    Y_valid = Y_flat[floor_mask]
    Z_valid = Z_flat[floor_mask]

    print(f"Generated {len(X_valid):,} reachable points.")

    # 5. Plotting the 3D Point Cloud
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    # Use a scatter plot with a very small point size (s=1) and low alpha for transparency
    scatter = ax.scatter(
        X_valid, Y_valid, Z_valid, c=Z_valid, cmap="viridis", s=1, alpha=0.1
    )

    # Formatting the plot
    ax.set_title("3-DOF Robotic Arm Reachable Workspace")
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.set_zlabel("Z (meters)")

    # Set equal axis limits to preserve the true physical shape
    max_reach = L1 + L2
    ax.set_xlim([-max_reach, max_reach])
    ax.set_ylim([-max_reach, max_reach])
    ax.set_zlim([0, L0 + max_reach])

    fig.colorbar(scatter, ax=ax, label="Z Height (meters)", shrink=0.5)
    plt.show()


# Run the simulation with your current arm dimensions
generate_workspace(L0=0.05, L1=0.1, L2=0.1)
