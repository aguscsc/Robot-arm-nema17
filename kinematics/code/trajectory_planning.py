import math
import matplotlib.pyplot as plt
import numpy as np

L_0 = 5
L_1 = 10
L_2 = 10


# --- IK Math & Pre-Flight Validator ---
def calculate_ik(x, y, z):
    r = math.sqrt(x**2 + y**2)
    h = z - L_0
    D = math.sqrt(r**2 + h**2)

    # Distance check
    if D > (L_1 + L_2):
        raise ValueError("Target is out of reach! (Exceeds physical workspace)")

    # Calculate angles in radians
    theta1 = math.atan2(y, x)

    # Shoulder angle (theta2)
    # Note: Using max/min bounds to prevent math domain errors from floating point inaccuracies
    cos_beta = max(-1.0, min(1.0, (L_1**2 + D**2 - L_2**2) / (2 * L_1 * D)))
    alpha = math.atan2(h, r)
    beta = math.acos(cos_beta)
    theta2 = alpha + beta

    # Elbow angle (theta3)
    cos_gamma = max(-1.0, min(1.0, (L_1**2 + L_2**2 - D**2) / (2 * L_1 * L_2)))
    gamma = math.acos(cos_gamma)
    theta3 = -(math.pi - gamma)  # External angle for the motor

    # Joint Limits Check
    limits = {
        "theta1": (-math.pi, math.pi),  # Base can rotate 360 (-180 to 180)
        "theta2": (0, math.pi / 2),  # Shoulder limits (0 to 90)
        "theta3": (-math.pi / 2, math.pi / 2),  # Elbow limits (-90 to 90)
    }

    if not (limits["theta1"][0] <= theta1 <= limits["theta1"][1]):
        raise ValueError(
            f"Base angle {np.degrees(theta1):.1f}° violates mechanical limits!"
        )

    if not (limits["theta2"][0] <= theta2 <= limits["theta2"][1]):
        raise ValueError(
            f"Shoulder angle {np.degrees(theta2):.1f}° violates mechanical limits!"
        )

    if not (limits["theta3"][0] <= theta3 <= limits["theta3"][1]):
        raise ValueError(
            f"Elbow angle {np.degrees(theta3):.1f}° violates mechanical limits!"
        )

    return np.array([theta1, theta2, theta3])


# --- Trajectory Generator ---
def generate_minimum_jerk_trajectory(
    start_angles, end_angles, total_time, time_step=0.01
):
    """
    Generates a synchronized Minimum Jerk (Quintic) trajectory for all joints.
    """
    # Generate time array
    t = np.arange(0, total_time + time_step, time_step)

    # Normalize time (tau goes from 0.0 to 1.0)
    tau = t / total_time

    # Calculate the S-Curve scaling factor (Quintic Polynomial)
    s_curve = 10 * tau**3 - 15 * tau**4 + 6 * tau**5

    # Calculate the delta for all joints
    delta = end_angles - start_angles

    # Vectorized broadcasting
    trajectory = (start_angles[:, np.newaxis] + delta[:, np.newaxis] * s_curve).T

    return t, trajectory


# --- Execution Pipeline ---
if __name__ == "__main__":
    coords = input("Insert target coordinates (x y z separated by spaces): ").split()

    if len(coords) == 3:
        x, y, z = float(coords[0]), float(coords[1]), float(coords[2])

        # Define Home Position (Where the robot wakes up)
        home_angles = np.array([0.0, math.pi / 2, 0.0])

        try:
            # Run the Pre-Flight Check & IK Validator
            print(f"Calculating trajectory to ({x}, {y}, {z})...")
            target_angles = calculate_ik(x, y, z)

            print(
                f"Target Validated! Final Angles (Degrees): {np.degrees(target_angles).round(2)}"
            )

            # Generate the Quintic S-Curve
            movement_duration = 2.0  # Seconds to complete the move
            t, trajectory = generate_minimum_jerk_trajectory(
                home_angles, target_angles, movement_duration
            )

            # Plot the Results
            plt.figure(figsize=(10, 6))

            # Convert radians to degrees for easier reading on the graph
            plt.plot(
                t, np.degrees(trajectory[:, 0]), label="Base (Theta 1)", linewidth=2
            )
            plt.plot(
                t, np.degrees(trajectory[:, 1]), label="Shoulder (Theta 2)", linewidth=2
            )
            plt.plot(
                t, np.degrees(trajectory[:, 2]), label="Elbow (Theta 3)", linewidth=2
            )

            plt.title("Quintic S-Curve Joint Trajectories (Minimum Jerk)")
            plt.xlabel("Time (seconds)")
            plt.ylabel("Joint Angle (Degrees)")
            plt.legend()
            plt.grid(True)
            plt.show()

        except ValueError as e:
            # This catches out-of-bounds math OR mechanical limit violations
            print(f"\n[PRE-FLIGHT ABORTED]: {e}")

    else:
        print("Please enter exactly three numbers.")
