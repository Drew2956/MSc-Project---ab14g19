% Import data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function process_transect_simulation (filename, row_skip = 0)
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
	depth = -data(:,4);
    prof  = -data (:,5);
    force = data (:,6);
	time = data (:,7);
    velocity = data (:,8);
	x =	data (:,9);

    N = size (data)(1)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Simulation wise parameters
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    time_step = 0.1         % seconds: dT solver
    control_time = 36000    % 10 hrs = 36.000 seconds in CONTROL mode
    target_altitude = 2.0   % target altitude employed for ALL the current simulation set

	printf ("%d entries loaded...\n", N)

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
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    power_t = force .* velocity;     % Power = Force x Velocity
    power_avg = mean (abs(power_t)); % Average power

    total_power = sum(abs(power_t)) * time_step;

    energy_t = force .* velocity * time_step; % Energy = Force x Distance = Force x Velocity x Time
    total_energy = sum(abs(energy_t)) * time_step; % Total Energy = integral (Energy . dt)

    plot (power_t)
    hold on 
    plot (energy_t, 'r')

    return
    altitude = depth - prof;

    close all
%    figure
%	hold on
%	plot (x, prof, 'k')
%    plot (x, prof + target_altitude, 'g--')
%    plot (x, depth)
%    grid on

    figure
    hold on
    plot (x, energy, 'r')
    %plot (x, target_altitude*ones(N,1), 'g')
    plot (x, thruster, 'k')
    grid on
    return


    % TODO: determine the system mode: DIVING, BRAKING, CONTROL, SURFACING
    % TODO: create issue in dritcam_simulator.py to add a vector column with information regarding the platform operatin mode
    % so this doesn't require postprocessing in this section of the pipeline

    % In order to detect the PHASE, we can use as input mission_time: 36.000 for CONTROL phase,
    % the start of ballast dispensing fror init BRAKING phase, and target_altitude for BRAKING->CONTROL
    % Also, the second ballast init, flags the SURFACING phase



	% Finally, we export the new data
	[fPath fName fExtension] = fileparts (filename)
	new_filename = strcat(fPath,"/",fName,"_interpolated.csv")

	dlmwrite(new_filename, out, 'delimiter', '\t');

