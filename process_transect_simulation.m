% Import data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [pm ps em es emx] = process_transect_simulation (filename, row_skip = 0, flag_plot = 1)
    DIVE = 0;
    BRAKE = 1;
    CONTROL = 2;
    SURFACE = 3;

    filename = strtrim(filename);   % remove any trailing space    
	printf ("Loading %s ...\n", filename)
	% Read data from filename, whose columns are separated by COMMA, and skip the first ROW (header)
	data = dlmread (filename, ',', 1 + row_skip, 0);
	% Expected data format, from Panda Export CSV. The column are sorted alphabetically (ask Panda why ¬¬)
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% acceleration, altitude, balls, depth, seafloor, thruster, time, velocity, x_position
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% [1] = Aceleration 	[m/s2]
	% [2] = Altitude 		[m]
	% [3] = # of Balls 		[units]
	% [4] = Depth 			[m]
	% [5] = Seafloor depth 	[m]
	% [6] = Thruster 		[N]
	% [7] = Time 			[s]
	% [8] = Vertical speed	[s]
	% [9] = X position		[m]
    n_balls = data(:, 3);
	depth = -data(:, 4);
    prof  = -data (:, 5);
    force = data (:, 6);
	time = data (:, 7);
    velocity = data (:, 8);
	x =	data (:, 9);

    N = size (data)(1)
    printf ("%d entries loaded...\n", N)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Simulation wise parameters
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    time_step = 0.1;         % seconds: dT solver
    mission_time = 360000;    % 100 hrs = 36.000 seconds in CONTROL mode, scaled to samples (dividing by 0.1)
    target_altitude = 2.0;   % target altitude employed for ALL the current simulation set

    altitude = depth - prof;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % THRUSTER MODEL:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %---For M100 with 3D printed propeller
    nominal_force = 1.1;     % [kg.f] Force of the thruster at the nominal operation point (usually, the most efficient point)
    nominal_force = nominal_force * 9.81;    % Convert from [kg.f] to [N]
    nominal_power = 80;     % [W] Power consumed when the thruster is at the nominal operation point

    %---For T100 thruster
    t100_factor = 42.637;    % [g.f / W] highest efficiency point for the T100 according to datasheet
    t100_factor = t100_factor * 9.81 / 1000; % convert from grams-force to Newton

    force_to_power = t100_factor;  % use the model factor. This can be changed according to the thruster employed

    power_required = abs(force) / force_to_power;     % instant power from the thruster to provide the required force 


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % EXTRACTING MODE INFORMATION FROM DEPTH AND N_BALLS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ball_dispense = n_balls(2:N) - n_balls(1:N-1);
    start_brake = find (ball_dispense, 1);  % find the first non-zero entry, meaning the start of the BRAKE phase, which is also the end of DIVING phase

    start_control = find((altitude < target_altitude), 1);

    start_surface = start_control + mission_time;

    % Now, we create a vector with the MODE information for each sample

    counter = [1:N];
    system_mode = (counter > start_brake) + (counter > start_control) + (counter > start_surface);

    % Now,we can mask and remove the nin-controlled phase for our error calculations
    mask = (system_mode == CONTROL)';
    altitude_error = (altitude - target_altitude) .* mask;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % DATA ANALYSIS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    power_mean = mean(abs(power_required(start_control:start_surface)))
    power_stdev = std (abs(power_required(start_control:start_surface)))

    altitude_error_mean = mean(abs(altitude_error(start_control:start_surface)))
    altitude_error_stdev = std(abs(altitude_error(start_control:start_surface)))

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Visualization
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    close all

    if (flag_plot == 1)

        figure
        title (filename)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Visualization
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        subplot (3,1,1)
            hold on

            plot (x, prof, 'k--')
            plot (x, depth, 'b', 'linewidth', 2)
            legend ("Seafloor[m]", "Depth[m]")

            % Improve data visualization by adding labels, and zoom box
            % Retrieve min/max values for the phases, adding some extra space at both sides
            _min = min (prof(start_brake:start_surface));
            _max = max (depth(start_brake:start_surface));
            _min = _min - abs(_min/200);
            _max = _max + abs(_max/200);

            % Improve data visualization by vertical dash lines indicating mode transition
            line ([x(start_brake) x(start_brake)], [_min _max], 'linestyle', '--', 'color', 'k')
            line ([x(start_control) x(start_control)], [_min _max], 'linestyle', '--', 'color', 'k')
            line ([x(start_surface) x(start_surface)], [_min _max], 'linestyle', '--', 'color', 'k')

            ylim ([_min _max])
            xlabel ("Distance [m]")

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Visualization
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        subplot (3,1,2)
            hold on

            plot (x, altitude_error, 'r', 'linewidth', 2)
            plot (x, velocity, 'g', 'linewidth', 2)
            legend ("Altitude error[m]", "Velocity[m/s]")

            % Retrieve min/max values for the phases, adding some extra space at both sides
            _min = min (altitude_error(start_control:start_surface));
            _max = max (altitude_error(start_control:start_surface));
            _min = _min - abs(_min/10);
            _max = _max + abs(_max/10);

            % Improve data visualization by vertical dash lines indicating mode transition
            line ([x(start_brake) x(start_brake)], [_min _max], 'linestyle', '--', 'color', 'k')
            line ([x(start_control) x(start_control)], [_min _max], 'linestyle', '--', 'color', 'k')
            line ([x(start_surface) x(start_surface)], [_min _max], 'linestyle', '--', 'color', 'k')

            ylim ([_min _max])
            xlabel ("Distance [m]")

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Visualization
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        subplot (3,1,3)
            hold on

            plot (x, power_required, 'b', 'linewidth', 2)
            legend ("Power[W]")

            % Retrieve min/max values for the phases, adding some extra space at both sides
            _min = min (power_required(start_control:start_surface));
            _max = max (power_required(start_control:start_surface));
            _min = _min - abs(_min/10);
            _max = _max + abs(_max/10);

            % Improve data visualization by vertical dash lines indicating mode transition
            line ([x(start_brake) x(start_brake)], [_min _max], 'linestyle', '--', 'color', 'k')
            line ([x(start_control) x(start_control)], [_min _max], 'linestyle', '--', 'color', 'k')
            line ([x(start_surface) x(start_surface)], [_min _max], 'linestyle', '--', 'color', 'k')

            ylim ([_min _max])
            xlabel ("Distance [m]")
        endif

    % WARNING: if we reverse signs for depth, we must do the same for the velocity.
    % WARNING: some resulting energy values are negative, meaning some of it could be recovered but our system is full dissipative
    % Check physical validation for our modeled system (E>0)
    % Review approx of integral to sum description of the equation

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % TODO: estimate Energy consumption
    % E = P.t
    % P = F.v
    % E = F.v.t = F.d
    % We have both the actual depth, and the instant velocity for each (t)
    % We should be able to estimate the distance traveled (dZ) for each interval (dT)
    % The time step is constant: 0.1 s for the existing dataset
    % WARNING: this calculations won't hold true as the hypothesis required is to be a conservative system
    % which is not our case, as there are dissipative forces (drag, i.e.)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
    % return values
    pm = power_mean;
    ps = power_stdev;

    em = altitude_error_mean;
    es = altitude_error_stdev;
    emx = max (abs(altitude_error(start_control:start_surface)));
    return
 

    % TODO: determine the system mode: DIVING, BRAKING, CONTROL, SURFACING

    % In order to detect the PHASE, we can use as input mission_time: 36.000 for CONTROL phase,
    % the start of ballast dispensing fror init BRAKING phase, and target_altitude for BRAKING->CONTROL
    % Also, the second ballast init, flags the SURFACING phase

	% Finally, we export the new data
	[fPath fName fExtension] = fileparts (filename)
	new_filename = strcat(fPath,"/",fName,"_interpolated.csv")

	dlmwrite(new_filename, out, 'delimiter', '\t');

