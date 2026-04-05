import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import scipy.io as sio

# --- Robot Physical Parameters ---
L_0 = 5
L_1 = 10
L_2 = 10


# --- IK Math & Pre-Flight Validator ---
def calculate_ik(x, y, z):
    r = math.sqrt(x**2 + y**2)
    h = z - L_0
    D = math.sqrt(r**2 + h**2)

    if D > (L_1 + L_2):
        raise ValueError("Target is out of reach! (Exceeds physical workspace)")

    # Base Angle
    theta1 = math.atan2(y, x)

    # Shoulder Angle (Math derivation)
    cos_beta = max(-1.0, min(1.0, (L_1**2 + D**2 - L_2**2) / (2 * L_1 * D)))
    alpha = math.atan2(h, r)
    beta = math.acos(cos_beta)
    theta2_math = alpha + beta

    # Elbow Angle (Math derivation)
    cos_gamma = max(-1.0, min(1.0, (L_1**2 + L_2**2 - D**2) / (2 * L_1 * L_2)))
    gamma = math.acos(cos_gamma)

    # --- MOTOR COORDINATE MAPPING ---
    # Shift theta2 so 0 degrees is straight UP (Z-axis)
    theta2 = (math.pi / 2) - theta2_math

    # Shift theta3 so 0 degrees is perfectly straight with the upper arm
    # Positive values bend the arm "forward/down"
    theta3 = math.pi - gamma

    # Adjusted Mechanical Limits (assuming 0 is straight up)
    limits = {
        "theta1": (-math.pi, math.pi),  # Base: -180 to 180
        "theta2": (0, math.pi / 2),  # Shoulder: 0 (Up) to 90 (Horizontal)
        "theta3": (
            -math.pi / 2,
            math.pi / 2,
        ),  # Elbow: -90 (Bent Back) to 90 (Bent Forward)
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
def generate_minimum_jerk_trajectory(start_angles, end_angles, time_step=0.02):
    delta = end_angles - start_angles
    max_delta = max(delta)
    # may change later to add speed profiles, for now 5 rad/s is the fastest the robot can go
    # also the peak velocity occurs at thau = 0.5
    total_time = 1.875 * max_delta / 3.0
    print(f"this will take {total_time} seconds")
    t = np.arange(0, total_time + time_step, time_step)
    tau = t / total_time
    s_curve = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    trajectory = (start_angles[:, np.newaxis] + delta[:, np.newaxis] * s_curve).T
    return t, trajectory


# --- Forward Kinematics (0 = Straight Up) ---
def get_joint_positions(theta1, theta2, theta3):
    # Origin
    x0, y0, z0 = 0, 0, 0
    # Shoulder Joint
    x1, y1, z1 = 0, 0, L_0

    # Elbow Joint (Notice sin and cos swapped for Z and XY mapping)
    x2 = L_1 * math.sin(theta2) * math.cos(theta1)
    y2 = L_1 * math.sin(theta2) * math.sin(theta1)
    z2 = L_0 + L_1 * math.cos(theta2)

    # Wrist/End Effector
    x3 = (L_1 * math.sin(theta2) + L_2 * math.sin(theta2 + theta3)) * math.cos(theta1)
    y3 = (L_1 * math.sin(theta2) + L_2 * math.sin(theta2 + theta3)) * math.sin(theta1)
    z3 = L_0 + L_1 * math.cos(theta2) + L_2 * math.cos(theta2 + theta3)

    return [x0, x1, x2, x3], [y0, y1, y2, y3], [z0, z1, z2, z3]


# --- Main Execution & Animation ---
if __name__ == "__main__":
    coords = input("Insert target coordinates (x y z separated by spaces): ").split()

    RECORD = int(input("RECORD yes(1)/no(0): "))
    EXPORT = int(input("EXPORT yes(1)/no(0): "))
    if len(coords) == 3:
        target_x, target_y, target_z = (
            float(coords[0]),
            float(coords[1]),
            float(coords[2]),
        )

        # PERFECTLY STRAIGHT UP ZERO REFERENCE
        home_angles = np.array([0.0, 0.0, 0.0])

        try:
            print(f"Calculating trajectory to ({target_x}, {target_y}, {target_z})...")
            target_angles = calculate_ik(target_x, target_y, target_z)

            print(
                f"Target Validated! Final Motor Angles (Degrees): {np.degrees(target_angles).round(2)}"
            )

            t, trajectory = generate_minimum_jerk_trajectory(
                home_angles,
                target_angles,
            )

            if EXPORT:
                # --- EXPORT TO SIMULINK ---
                # Simulink expects a matrix where Column 0 is time, and subsequent columns are data.
                simulink_matrix = np.column_stack((t, trajectory))

                # Save it as a MATLAB workspace file
                sio.savemat(
                    "s_curve_trajectory.mat", {"sim_trajectory": simulink_matrix}
                )
                print("\n[SUCCESS] Trajectory exported to 's_curve_trajectory.mat'")

            fig = plt.figure(figsize=(8, 8))
            ax = fig.add_subplot(111, projection="3d")

            ax.set_xlim([-20, 20])
            ax.set_ylim([-20, 20])
            ax.set_zlim([0, 30])
            ax.set_xlabel("X Axis")
            ax.set_ylabel("Y Axis")
            ax.set_zlabel("Z Axis")
            ax.set_title("S-Curve Motion (Z-Axis Zero Reference)")

            (line,) = ax.plot([], [], [], "o-", lw=4, markersize=8, color="#2ca02c")

            target_xs, target_ys, target_zs = get_joint_positions(
                target_angles[0], target_angles[1], target_angles[2]
            )
            ax.plot(
                target_xs,
                target_ys,
                target_zs,
                "o--",
                lw=2,
                markersize=5,
                color="gray",
                alpha=0.5,
                label="Target",
            )
            ax.legend()

            def init():
                line.set_data([], [])
                line.set_3d_properties([])
                return (line,)

            def update(frame):
                th1, th2, th3 = trajectory[frame]
                xs, ys, zs = get_joint_positions(th1, th2, th3)
                line.set_data(xs, ys)
                line.set_3d_properties(zs)
                return (line,)

            ani = animation.FuncAnimation(
                fig,
                update,
                frames=len(trajectory),
                init_func=init,
                blit=True,
                interval=20,
                repeat_delay=1000,
            )

            if RECORD:
                ani = animation.FuncAnimation(
                    fig,
                    update,
                    frames=len(trajectory),
                    init_func=init,
                    blit=True,
                    interval=20,
                    repeat_delay=1000,
                )

                print(
                    "Saving 's_curve_simulation.gif'... (This might take a few seconds)"
                )
                # The 'pillow' writer is built into matplotlib's standard dependencies
                ani.save("s_curve_simulation.gif", writer="pillow", fps=50)
                print("[SUCCESS] GIF saved successfully!")

            plt.show()
            plt.show()

        except ValueError as e:
            print(f"\n[PRE-FLIGHT ABORTED]: {e}")

    else:
        print("Please enter exactly three numbers.")
