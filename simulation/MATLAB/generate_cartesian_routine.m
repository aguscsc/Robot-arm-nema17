function [ts_x, ts_y, ts_z] = generate_cartesian_routine()
    % Generates a smooth 11-second Cartesian trajectory (in meters)
    
    dt = 0.001;          
    t = (0:dt:11)';       
    N = length(t);
    
    x = zeros(N, 1);
    y = zeros(N, 1);
    z = zeros(N, 1);
    
    % --- TRAJECTORY CONTROL PANEL ---
    % Change these variables to shift the entire routine in 3D space
    home_x = 0.05;
    home_y = 0.00;
    home_z = 0.15;
    
    square_start_x = 0.10;
    square_start_y = -0.025;
    square_start_z = 0.05;
    
    for i = 1:N
        time = t(i);
        
        if time < 1.0
            % PHASE 0: Initialize and Hold at Home (0 to 1s)
            % Gives the digital twin and controllers time to settle
            x(i) = home_x;
            y(i) = home_y;
            z(i) = home_z;
            
        elseif time >= 1.0 && time < 3.0
            % PHASE 1: Smooth Approach (1 to 3s)
            norm_t = (time - 1.0) / 2.0;
            smooth = 0.5 - 0.5 * cos(pi * norm_t);
            
            x(i) = home_x + (square_start_x - home_x) * smooth;
            y(i) = home_y + (square_start_y - home_y) * smooth;
            z(i) = home_z + (square_start_z - home_z) * smooth;
            
        elseif time >= 3.0 && time < 4.0
            % PHASE 2a: Square Bottom Edge (Move Left)
            norm_t = (time - 3.0) / 1.0;
            smooth = 0.5 - 0.5 * cos(pi * norm_t);
            
            x(i) = square_start_x;
            y(i) = square_start_y + (0.025 - square_start_y) * smooth;
            z(i) = square_start_z;
            
        elseif time >= 4.0 && time < 5.0
            % PHASE 2b: Square Right Edge (Move Up)
            norm_t = (time - 4.0) / 1.0;
            smooth = 0.5 - 0.5 * cos(pi * norm_t);
            
            x(i) = square_start_x;
            y(i) = 0.025;
            z(i) = square_start_z + (0.10 - square_start_z) * smooth;
            
        elseif time >= 5.0 && time < 6.0
            % PHASE 2c: Square Top Edge (Move Right)
            norm_t = (time - 5.0) / 1.0;
            smooth = 0.5 - 0.5 * cos(pi * norm_t);
            
            x(i) = square_start_x;
            y(i) = 0.025 + (-0.025 - 0.025) * smooth;
            z(i) = 0.10;
            
        elseif time >= 6.0 && time < 7.0
            % PHASE 2d: Square Left Edge (Move Down)
            norm_t = (time - 6.0) / 1.0;
            smooth = 0.5 - 0.5 * cos(pi * norm_t);
            
            x(i) = square_start_x;
            y(i) = -0.025;
            z(i) = 0.10 + (square_start_z - 0.10) * smooth;
            
        elseif time >= 7.0 && time < 9.0
            % PHASE 3: The Heavy Swing (Arc base while extended)
            norm_t = (time - 7.0) / 2.0;
            smooth = 0.5 - 0.5 * cos(pi * norm_t);
            
            x(i) = square_start_x + (0.05 - square_start_x) * smooth;
            y(i) = -0.025 + (0.10 - -0.025) * smooth;
            z(i) = square_start_z;
            
        else
            % PHASE 4: Retract to Home (9 to 11s)
            norm_t = (time - 9.0) / 2.0;
            smooth = 0.5 - 0.5 * cos(pi * norm_t);
            
            x(i) = 0.05 + (home_x - 0.05) * smooth;
            y(i) = 0.10 + (home_y - 0.10) * smooth;
            z(i) = 0.05 + (home_z - 0.05) * smooth;
        end
    end
    
    % Convert raw arrays into Simulink Timeseries objects
    ts_x = timeseries(x, t);
    ts_y = timeseries(y, t);
    ts_z = timeseries(z, t);
    
    disp('Cartesian trajectory successfully generated with Home variables!');
end