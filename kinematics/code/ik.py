import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation

RECORD = 0
L_0 = 5
L_1 = 10
L_2 = 10


# ---IK Math ---
def calculate_ik(x, y, z):
    r = math.sqrt(x**2 + y**2)
    h = z - L_0
    D = math.sqrt(r**2 + h**2)

    # Safety Check: Is it too far?
    if D > (L_1 + L_2):
        print("Error: Target is out of reach!")
        return None

    # Calculate angles in radians
    theta1 = math.atan2(y, x)

    # Shoulder angle (theta2)
    alpha = math.atan2(h, r)
    beta = math.acos((L_1**2 + D**2 - L_2**2) / (2 * L_1 * D))
    theta2 = alpha + beta

    # Elbow angle (theta3)
    gamma = math.acos((L_1**2 + L_2**2 - D**2) / (2 * L_1 * L_2))
    theta3 = -(math.pi - gamma)  # External angle for the motor

    # Convert to degrees for easier limit checking
    # t1_deg = math.degrees(theta1)
    t2_deg = math.degrees(theta2)
    t3_deg = math.degrees(theta3)
    print(f"theta1: {theta1}, theta2: {theta2}, theta3: {theta3}")
    # Joint Limits Check
    if t3_deg < -150 or t3_deg > 150:
        print(f"Error: Elbow hyperextension ({t3_deg:.1f} deg)")
        return None
    if t2_deg < 0 or t2_deg > 175:
        print(f"Error: Shoulder hyperextension ({t2_deg:.1f} deg)")
        return None

    return theta1, theta2, theta3


# --- Forward Kinematics for Plotting ---
def get_joint_positions(th1, th2, th3):
    # Joint 0: Origin
    x0, y0, z0 = 0, 0, 0
    # Joint 1: Shoulder
    x1, y1, z1 = 0, 0, L_0
    # Joint 2: Elbow
    x2 = L_1 * math.cos(th2) * math.cos(th1)
    y2 = L_1 * math.cos(th2) * math.sin(th1)
    z2 = L_0 + L_1 * math.sin(th2)
    # Joint 3: End Effector
    x3 = x2 + L_2 * math.cos(th2 + th3) * math.cos(th1)
    y3 = y2 + L_2 * math.cos(th2 + th3) * math.sin(th1)
    z3 = z2 + L_2 * math.sin(th2 + th3)

    return [x0, x1, x2, x3], [y0, y1, y2, y3], [z0, z1, z2, z3]


# --- 3D Animation ---
def animate_arm(target_x, target_y, target_z):
    angles = calculate_ik(target_x, target_y, target_z)
    if not angles:
        return

    target_th1, target_th2, target_th3 = angles

    # Set starting "Home" position (standing straight up)
    start_th1, start_th2, start_th3 = 0, math.pi / 2, 0

    # Create the animation frames (Sequential movement)
    frames = []
    steps = 20

    # Phase 1: Rotate Base (Theta 1)
    for i in range(steps):
        t = i / float(steps - 1)
        th1 = start_th1 + t * (target_th1 - start_th1)
        frames.append((th1, start_th2, start_th3))

    # Phase 2: Rotate Shoulder (Theta 2)
    for i in range(steps):
        t = i / float(steps - 1)
        th2 = start_th2 + t * (target_th2 - start_th2)
        frames.append((target_th1, th2, start_th3))

    # Phase 3: Rotate Elbow (Theta 3)
    for i in range(steps):
        t = i / float(steps - 1)
        th3 = start_th3 + t * (target_th3 - start_th3)
        frames.append((target_th1, target_th2, th3))

    # Setup the plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    (line,) = ax.plot([], [], [], "o-", lw=4, markersize=8, color="blue")
    target_scatter = ax.scatter(
        [target_x], [target_y], [target_z], color="red", s=50, label="Target"
    )

    ax.set_xlim(-25, 25)
    ax.set_ylim(-25, 25)
    ax.set_zlim(0, 30)
    ax.set_xlabel("X Axis")
    ax.set_ylabel("Y Axis")
    ax.set_zlabel("Z Axis")
    ax.set_title("3-DOF Arm Kinematics")
    ax.legend()

    def update(frame):
        th1, th2, th3 = frame
        xs, ys, zs = get_joint_positions(th1, th2, th3)
        line.set_data(xs, ys)
        line.set_3d_properties(zs)
        return (line,)

    if RECORD:
        ani = animation.FuncAnimation(
            fig, update, frames=frames, interval=50, blit=False, repeat=False
        )

        # --- The Recording Logic ---
        print("Saving animation to GIF... (This might take a few seconds)")

        # writer='pillow' is the magic word for GIFs
        # fps=20 matches the 50ms interval (1000ms / 50ms = 20 fps)
        ani.save("ik_simulation.gif", writer="pillow", fps=20)

        print("Saved successfully as ik_simulation.gif!")

        # You can still show it on screen after it saves
        plt.show()
    else:
        ani = animation.FuncAnimation(
            fig, update, frames=frames, interval=50, blit=False, repeat=False
        )
        plt.show()


if __name__ == "__main__":
    RECORD = int(input("do you want to record the movement? Y(1) / N(0) "))
    coords = input("Insert target coordinates (x y z separated by spaces): ").split()
    if len(coords) == 3:
        x, y, z = float(coords[0]), float(coords[1]), float(coords[2])
        animate_arm(x, y, z)
    else:
        print("Please enter exactly three numbers.")
