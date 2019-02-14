data = dlmread ('data/simulations/batch/batch_e0.03_d0.016_cCTD01_tT1.csv', ',', 1, 0);

DIVE = 0;
BRAKE = 1;
CONTROL = 2;
SURFACE = 3;

data_s = data (1:10:end,:);

N =length (data_s)
n_balls = data_s (:,3);
altitude = data_s (:, 2);
depth = data_s (:, 4);
seafloor = data_s (:,5);
force = data_s (:,6);
time = data_s (:, 7);
x = data_s (:,9);

t100_factor = 42.637;    % [g.f / W] highest efficiency point for the T100 according to datasheet
t100_factor = t100_factor * 9.81 / 1000; % convert from grams-force to Newton
force_to_power = t100_factor;  % use the model factor. This can be changed according to the thruster employed
power_required = abs(force) / force_to_power;     % instant power from the thruster to provide the required force 


close all
subplot (3,1,1)
h = plot (x, depth, "linewidth", 1);
set (gcf, "position", [500 1000 1000 720])

ax1 = gca()

set (ax1, "linewidth", 1)
set (ax1, "fontname", "arial")
set (ax1, "fontsize", 16)
set (ax1, "labelfontsizemultiplier", 1.2)
set (ax1, "minorgridlinestyle", ":")
set (ax1, "yminorgrid", "off")
set (ax1, "ygrid", "on")

grid on
hold on

% Reverse axis
set (ax1, "Ydir", "reverse")
ylim ([500 1000])
xlim ([0 2100])

title ("Dive profile: Transect 1 - CTD 01 - Ballast size: 16mm")
ylabel ("Depth (m)")

plot (x, seafloor, "r--", "linewidth", 0.5);

lgd1 = legend ("Depth (m)", "Seafloor (m)")
set (lgd1, "fontsize", 18)
set (lgd1, "fontname", "arial")
set (lgd1, "location", "southeast")


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
target_altitude = 2.0;
mission_time = 3600;
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOT THRUSTER
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot (3,1,2)
ax2 = gca()
plot (x, power_required, "linewidth", 1)
%xlabel ("Distance (m)")

set (ax2, "linewidth", 1)
set (ax2, "fontname", "arial")
set (ax2, "fontsize", 16)
set (ax2, "labelfontsizemultiplier", 1.2)
set (ax2, "minorgridlinestyle", ":")
set (ax2, "yminorgrid", "off")
set (ax2, "ygrid", "on")

ylabel ("Thruster power (W)")
xlim ([0 2100])

lgd2 = legend ("Thruster power (W)")
set (lgd2, "fontsize", 18)
set (lgd2, "fontname", "arial")
set (lgd2, "location", "southeast")

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOT ERROR
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot (3,1,3)
ax3 = gca()
plot (x, altitude_error, "r", "linewidth", 1)

set (ax3, "linewidth", 1)
set (ax3, "fontname", "arial")
set (ax3, "fontsize", 16)
set (ax3, "labelfontsizemultiplier", 1.2)
set (ax3, "minorgridlinestyle", ":")
set (ax3, "yminorgrid", "off")
set (ax3, "ygrid", "on")

xlabel ("Distance (m)")
ylabel ("Altitude error (m)")
xlim ([0 2100])

% Create a small zoom box
_min = -1
_max =  1
_left = 160;
_right = 180;
line ([_left _left],[_min _max], "linestyle", "--", "color", 'k')
line ([_right _right],[_min _max], "linestyle", "--", "color", 'k')
ylim ([-0.15 0.05])

rectangle ()

lgd3 = legend ("Altitude error (m)")
set (lgd3, "fontsize", 18)
set (lgd3, "fontname", "arial")
set (lgd3, "location", "southeast")

ax4 = axes ("Position", [0.23 0.133 0.25 0.23]);
%set (ax4, "position", pos + [0 0 0 0])
plot (x, altitude_error, "r", "linewidth", 1)
grid on
xlim ([_left _right])
