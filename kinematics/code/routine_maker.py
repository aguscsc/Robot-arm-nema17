import s_curve
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import scipy.io as sio
import math
if __name__ == "__main__":
    points_str = input("How many points should the robot hit: ")
    points = int(points_str)

    routine_trajectories = []
    
    # Set the absolute starting physical position once
    initial_home_angles = np.array([0.0, math.pi/2, 0.0])
    current_angles = np.copy(initial_home_angles)
    
    # Define the time step globally so Simulink and Matplotlib sync perfectly
    TIME_STEP = 0.02 

    # Input Loop (Only advances when a point is valid)
    valid_points = 0
    while valid_points < points:
        coords = input(f"Point {valid_points+1} - Insert target coordinates (x y z): ").split()
        
        if len(coords) == 3:
            try:
                target_x = float(coords[0])
                target_y = float(coords[1])
                target_z = float(coords[2])

                print(f"Calculating trajectory to ({target_x}, {target_y}, {target_z})...")
                
                # Pre-Flight Check
                target_angles = s_curve.calculate_ik(target_x, target_y, target_z)
                print(f"Target Validated! Final Motor Angles: {np.degrees(target_angles).round(2)}°")

                # Generate path
                t, trajectory = s_curve.generate_minimum_jerk_trajectory(
                    current_angles,
                    target_angles,
                    time_step=TIME_STEP
                )
                
                # Append to master list and update memory
                routine_trajectories.append(trajectory)
                current_angles = target_angles 
                
                # Successfully calculated! Increment the counter to move to the next point
                valid_points += 1 
            
            except ValueError as e:
                # The script caught an impossible move, but will NOT break the loop.
                print(f"\n[PRE-FLIGHT ABORTED]: {e}")
                print("Please enter a valid, physically reachable coordinate to continue.")
        else:
            print("Please enter exactly three numbers.")

    # --- Merge, Export, and Animate the Full Routine ---
    if routine_trajectories:
        # Stack all individual movements into one master array
        full_trajectory = np.vstack(routine_trajectories)
        total_waypoints = len(full_trajectory)
        print(f"\n[SUCCESS] Full routine generated! Total waypoints: {total_waypoints}")

        # --- EXPORT TO MATLAB/SIMULINK ---
        save_mat = input("Do you want to save the routine for Simulink (.mat)? Y(1) / N(0): ").strip()
        if save_mat == '1':
            # Create a monolithic time array (0.00, 0.02, 0.04...) for the entire routine
            t_full = np.arange(total_waypoints) * TIME_STEP
            simulink_matrix = np.column_stack((t_full, full_trajectory))
            
            sio.savemat('routine_trajectory.mat', {
                'sim_trajectory': simulink_matrix,
                'sim_time': t_full[-1],
                'init_base': initial_home_angles[0],
                'init_shoulder': initial_home_angles[1],
                'init_elbow': initial_home_angles[2]
            })
            print("[SUCCESS] Exported to 'routine_trajectory.mat'")

        # --- SETUP ANIMATION ---
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        ax.set_xlim([-20, 20])
        ax.set_ylim([-20, 20])
        ax.set_zlim([0, 30])
        ax.set_xlabel('X Axis')
        ax.set_ylabel('Y Axis')
        ax.set_zlabel('Z Axis')
        ax.set_title("Multi-Point Continuous Path Simulation")
        
        line, = ax.plot([], [], [], 'o-', lw=4, markersize=8, color='#2ca02c')

        def init():
            line.set_data([], [])
            line.set_3d_properties([])
            return line,

        def update(frame):
            th1, th2, th3 = full_trajectory[frame]
            xs, ys, zs = s_curve.get_joint_positions(th1, th2, th3) 
            line.set_data(xs, ys)
            line.set_3d_properties(zs)
            return line,

        ani = animation.FuncAnimation(fig, update, frames=total_waypoints, init_func=init, blit=True, interval=20, repeat_delay=1000)
        
        # --- GIF RECORDING ---
        save_gif = input("Do you want to record the movement to a GIF? Y(1) / N(0): ").strip()
        if save_gif == '1':
            print("Saving 'routine_simulation.gif'... (This might take a few seconds)")
            ani.save('routine_simulation.gif', writer='pillow', fps=50)
            print("[SUCCESS] GIF saved successfully!")
            
        plt.show()
