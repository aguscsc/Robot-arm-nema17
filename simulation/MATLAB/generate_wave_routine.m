function [ts_base, ts_shoulder, ts_elbow] = generate_wave_routine()
    % Generates a smooth 5-second waving trajectory for Simulink
    
    dt = 0.001;          % 1000 Hz sample time to match your discrete solver
    t = (0:dt:5)';       % Time vector from 0 to 5 seconds
    
    % Initialize empty arrays
    q_base = zeros(size(t));
    q_shoulder = zeros(size(t));
    q_elbow = zeros(size(t));
    
    for i = 1:length(t)
        time = t(i);
        
        % 1. BASE: Keep it locked facing forward (0 radians)
        q_base(i) = 3*pi/4; 
        
        % 2. SHOULDER: Smoothly lift to pi/3, hold, then lower
        if time < 1.0
            % Phase 1: Smooth ramp up using half-cosine (0 to 1 sec)
            q_shoulder(i) = (pi/3) * (0.5 - 0.5*cos(pi * time));
            
        elseif time >= 1.0 && time <= 4.0
            % Phase 2: Hold at pi/3 (1 to 4 sec)
            q_shoulder(i) = pi/3;
            
        else
            % Phase 3: Smooth ramp back down to 0 (4 to 5 sec)
            q_shoulder(i) = (pi/3) * (0.5 + 0.5*cos(pi * (time - 4.0)));
        end
        
        % 3. ELBOW: Wait for the shoulder to lift, then wave 3 times
        if time >= 1.0 && time <= 4.0
            % 3 full waves over 3 seconds (1 Hz frequency)
            wave_amplitude = pi/4; 
            q_elbow(i) = wave_amplitude * sin(2 * pi * 1.0 * (time - 1.0));
        else
            q_elbow(i) = 0; % Keep straight while lifting and lowering
        end
    end
    
    % Convert raw arrays into Simulink Timeseries objects for perfect playback
    ts_base = timeseries(q_base, t);
    ts_shoulder = timeseries(q_shoulder, t);
    ts_elbow = timeseries(q_elbow, t);
    
    disp('Waving trajectory successfully generated!');
end