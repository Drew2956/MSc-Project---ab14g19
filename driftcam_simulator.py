# Driftcam diving simulator. 
# Created by: M. Massot
# Edited by: M. Massot, J. Cappelletto

import time
import math
import pandas as pd
import yaml
import plotly.offline as py
from plotly import tools
import plotly.graph_objs as go
from SimplePID import SimplePID

print ("Driftcam diving simulator")
print ("Loading configuration.yaml...")
config_file = "configuration.yaml"

start_time = time.time()

#######################################
# Read configuration parameters from configuration.yaml
with open(config_file,'r') as stream:
    configuration = yaml.load(stream)   

print ("Loading density profile from: ", configuration['density_profile'])
# read input data (density profile)
density_table = pd.read_csv(configuration['density_profile'], sep='\t', header=None)
#print (density_table)
#time.sleep(5)
# This file must an already interpolated dataset, at 1.0 m depth resolution for faster calculations
# By doing this, the interpolation is done via lookup table, starting from 0 meter index

# Starting values for the simulation
depth = configuration['input']['start_depth']  # initial simulation depth
vertical_velocity = configuration['input']['start_velocity']   # initial vertical velocity. For faithfull simulations, it should be close to 0.0
horizontal_position = configuration['transect']['horizontal_position_offset']   # starting point in X axis for the transect mode simulations
t = 0.0
dropped_ball_time = 0   # timestamp of last dropped ball event

# Use first value from the lookup table. In fixed seawater simulations, this will be the constant value to be used
seawater_density = density_table[1][int (math.floor(depth))]
eta = configuration['input']['eta']

# Solver parameters
time_step = configuration['input']['time_step']     # Euler solver time step. Ideal step values: 0.1 , as smaller steps doesn't provide any noticeable change in system dynamics response
simulation_time = configuration['input']['simulation_time']  # End of simulation time
min_dispensing_time = configuration['input']['min_dispensing_time'] # minimum admissible time between drop ball event

# define platform specific parameters
gravity = configuration['input']['gravity'] # this could be compensated according to the latitude and the geodesic model
drag_coefficient = configuration['input']['drag_coefficient']   # geometry defined drag coefficient of the platform
radius = configuration['input']['radius']   # platform radius cross-section radius
area = (math.pi*radius**2)  # cross-section area
tau = configuration['input']['tau'] # system dynamic response constant

main_mass = configuration['input']['main_mass'] # kg
main_volume = configuration['input']['main_volume'] # m3

ball_diameter = configuration['input']['ball_diameter'] # ballast unit diameter
ball_volume = 4/3*math.pi*math.pow(ball_diameter/2,3)   # ballast unit volume
ball_density = configuration['input']['ball_density'] # kg / m3 # ball_mass = 0.016728 # kg 
ball_mass = ball_volume * ball_density  # ballast unit mass

flotation_mass = configuration['input']['flotation_mass'] # kg
flotation_density = configuration['input']['flotation_density'] # kg/m3
flotation_volume = flotation_mass / flotation_density   # total volume of the flotation mass (assumed to be constant)

ballast_mass = configuration['input']['ballast_mass']   # design defined initial ballast_mass
number_of_balls = math.ceil(ballast_mass / ball_mass)   # rounded-up number of ballast units based on the desired ballast_mass and the ballas unit mass

# TODO: floor can be a fixed value or a transect profile (See #4)
braking_altitude = configuration['control']['braking_altitude']
mission_duration = configuration['control']['mission_duration']

floor_model = configuration['floor_model']  # retrieve floor model: fixed or transect
if floor_model == 'fixed':
    print ("Using 'floor_model': ", floor_model)
    floor_profundity = configuration['control']['floor_profundity']
elif floor_model == 'transect':
    print ("Importing transect profile from: ", configuration['transect']['transect_profile'])
    profundity_table = pd.read_csv(configuration['transect']['transect_profile'], sep='\t', header=None)
    floor_profundity = profundity_table[1][0] + configuration['transect']['transect_profundity_offset']
else:
    print ("Unknown 'floor_model' ", floor_model, " defined in 'configuration.yaml'")
    print ("Using default 'fixed' depth model with depth = 100m")
    floor_model = 'fixed'
    floor_profundity = 100

_t1_floor_profundity = floor_profundity
_t0_floor_profundity = floor_profundity
#######################################
# Print summary of simulation parameters
print ("----------------------------------")
print ("Platform parameters:")
print (" * Plaform:")
print ("\tMass[kg]: ", main_mass, "\tVolume[m3]: ", main_volume)
print (" * Flotation:")
print ("\tMass[kg]: ", flotation_mass, "[kg]\tVolume[m3]: ", flotation_volume)
print (" * Ballast:")
print ("\tMass[kg]: ", ballast_mass, "\tDiameter[mm]: ", 1000*ball_diameter, "\tNumber: ", str(number_of_balls), "\tETA: ", eta)

print ("++++++++++++++++++++++++++++++++++")
print ("Simulation parameters:")
print (" * Initial conditions:")
print ("\tDepth[m]: ", str(depth), "\tVelocity[m/s]: ", str(vertical_velocity))
print (" * Target:")
print ("\tAltitude[m]: ", str(braking_altitude), "\tProfundity[m]: ", floor_profundity, "\tMission duration [s]: ", mission_duration)
print (" * Solver:")
print ("\tTime step[s]: ", time_step, "\tGravity[m/s2]: ", gravity, "\tDrag coeff: ", drag_coefficient)

# string used for the output file name (both CSV and HTML outputs)
density_id = configuration['density_profile'].split("_")[0]
density_id = density_id.split("/")[-1]

if floor_model == 'fixed':
    floor_id = str(floor_profundity)
elif floor_model == 'transect':
    floor_id = configuration['transect']['transect_profile'].split("_")[0]
    floor_id = floor_id.split("/")[-1]

simulation_details = "_e" + str(eta) + "_d" + str(ball_diameter) + "_c" + str(density_id) + "_t" + floor_id

#  Empty placeholders for incoming simulation data
time_dive_history = []
velocity_dive_history = []
horizontal_position_history = []
floor_profundity_history = []
depth_dive_history = []
error_history = []
altitude_history = []
net_buoyancy_dive_history = []
drag_dive_history = []
balls_history = []
acceleration_dive_history = []
thruster_history = []

last_velocity = vertical_velocity

print ("\nRunning simulation ...\n--------------------")

print ("\nStarting DIVING mode")
print ("Time: ", t, "\tDepth: ", depth)
mode = 'DIVING'

thruster_force = 0  # starting with the thrusters OFF
safety_factor_altitude = 1.0    # safety margin to trigger phase transition from BRAKING to CONTROL (basically increases the Htarget for the control, so it fires the thrusters before reaching the target)
target_altitude = configuration['control']['target_altitude']   # reference value for the altitude controller

_kp = configuration['control']['kp_gain']
_ki = configuration['control']['ki_gain']
_kd = configuration['control']['kd_gain']

# creates SimplePID controller object, with defined gains and hard limits for the control output
pid = SimplePID(target_altitude, -100, 100, _kp, _ki, _kd, time_step * 1000 )

keep_running = True

#while t < simulation_time:     # limit simulation time and depth (due to speed constraints)
#while (depth < (floor_profundity + 10)) and (t < simulation_time) and (keep_running == True):     # limit simulation time and depth (due to speed constraints)
while (t < simulation_time) and (keep_running == True):     # limit simulation time and depth (due to speed constraints)

    if (depth > (floor_profundity + 10)):
        print ("\n>>>>>>>>>\tPlatform crashed!!!")
        print ("\n>>>>>>>>>\tHalting simulation...")
        print (floor_profundity)
        print (depth)
        keep_running = False
        break
    ################################################
    # HORIZONTAL MOVEMENT SIMULATION
    # Only required for floor_model = 'transect' mode
    ################################################
    horizontal_position = horizontal_position + configuration['transect']['horizontal_speed']*time_step

    ################################################
    # FLOOR MODEL UPDATE
    # This will affect floor_profundity depending on the current seafloor model
    ################################################
    # if current model is transect, update according to the transect table
    if floor_model == 'transect':
        # the new profundity value will be the profile value for the current horizontal position + the profundity offset
        # TODO: try/catch if we exceed the maximum transect length
        # WARNING: cheap and dirty low pass filter to avoid abrupt references changes
        # that will trigger spikes in the controller
        _t2_floor_profundity = _t1_floor_profundity
        _t1_floor_profundity = floor_profundity
        # when indexing, horizontal_position is converted from input file resolution (cm) to its close integer index, for faster lookup
        _t0_floor_profundity = profundity_table[1][math.floor(horizontal_position*100)] + configuration['transect']['transect_profundity_offset']

        floor_profundity = 0.05*_t0_floor_profundity + 0.9*_t1_floor_profundity + 0.05 *_t2_floor_profundity
    # otherwise, we keep using the same initial value

    ################################################
    # MASS AND BUOYANCY UPDATE
    ################################################
    # replace weight with Mass
    microballast_mass = ball_mass * number_of_balls # kg
    microballast_volume = ball_volume * number_of_balls # m3

    # complete system volume: platform + flotation + microballasts
    total_volume = flotation_volume + main_volume + microballast_volume
    total_mass = main_mass + microballast_mass + flotation_mass
    total_weight = total_mass*gravity

    # See if it is possible to improve speed while doing the density search in the input vector (already ordered)
    seawater_density = density_table[1][int (math.floor(depth))]

    ball_buoyancy = ball_volume*gravity*seawater_density    # weight of the displaced seawater volume (per ball)

    ################################################
    ## BODY-FLUID INTERACTION SIMULATION
    ################################################
    buoyancy_force = total_volume*gravity*seawater_density
    drag_force = 0.5*drag_coefficient*seawater_density*area*abs(vertical_velocity)*vertical_velocity
    # calculate the actual net force experienced by the platform
    net_force = total_weight - buoyancy_force - drag_force - thruster_force
    # The real acceleration is obtained from F = m . a 
    acceleration = net_force / total_mass
    # The velocity is calculated via Euler integration for the acceleration 
    vertical_velocity = vertical_velocity + acceleration*time_step
#    force_drag = total_weight - force_buoyancy  # assume it acelerates to terminal velocity
    depth = depth + vertical_velocity*time_step

    # to avoid non-valid cases, such as emerging from the water
    if depth < 0:
        print ("Simulation halted: DEPTH < 0\n\tPlatform surfaced at ", t, " seconds")
        keep_running = False
        break

    ################################################
    ## FUTURE VELOCITY ESTIMATOR
    ################################################
    future_velocity = vertical_velocity + 2*tau*(vertical_velocity-last_velocity)

    # Here we check if we need to dispense a ball, the conditions are:
    # - The number of available balls is still positive
    # - The time since last ball dispensed is higher than our minimum dispensing time
    # - The estimated future velocity will be positive (we can keep diving)
    # - Depth > some reference value, this is temporal, as we will trigger the diving / braking transition given the altitude (H)

    # TODO: estimate power consumption for each dispensing action (depth invariant, just need to multiply by the expected power consumed per ball drop action)
    current_altitude = floor_profundity - depth
    altitude_error = 0  # this is and ampty value to fill the data vector when mode is NOT CONTROL

    # ------------------------------------------------------------------------------------------------------
    # DIVING MODE: default starting mode
    if mode == 'DIVING':
        # Check if current altitude is lower than braking_altitude. If so, we switch to BRAKING mode
        if (current_altitude < braking_altitude):
            print ("Starting BRAKING procedure mode")
            print ("Time: ", t, "\tDepth: ", depth)
            mode = 'BRAKING'

    # ------------------------------------------------------------------------------------------------------
    # BRAKING MODE: dispensing balls to achieve close to neutral buoyancy, dropping diving ballast (wd)
    elif mode == 'BRAKING':
        if (current_altitude < (target_altitude * safety_factor_altitude)):
            print ("Starting CONTROL mode")
            mode = 'CONTROL'
            start_control_time = t  # use current timestamp as start time for the control phase
            print ("Time: ", t, "\tDepth: ", depth)
        else:
            # If we haven't reached the target_altitude, we check if we can keep dispensing balls
            # The conditions are:
            # - Number of balls still positive
            # - Elapsed time since last drop > minimum dispensing time (defined in the configuration)
            # - Estimated new velocity is still positive, i.e. it will keep diving with the current density and ballast
            if (number_of_balls > 0) and (t - dropped_ball_time) > min_dispensing_time and (future_velocity > 0):
#            if (number_of_balls > 0) and (t - dropped_ball_time) > min_dispensing_time and (acceleration > 0):
                #  reset dropped ball timestamp
                dropped_ball_time = t
                # drop a single ball
                number_of_balls -= 1
                # update last reference vertical velocity, that is employed for future velocity estimation
                last_velocity = vertical_velocity

    # ------------------------------------------------------------------------------------------------------
    # CONTROL MODE: active altitude control with the thruster
    elif mode == 'CONTROL':
        altitude_error =  current_altitude - target_altitude
        #thruster_force = -_kp*altitude_error 
        # if abs(altitude_error) > 0.1:
        # print ("Controlling...........")
        
        last_thruster_force = thruster_force
        pid_out = pid.get_output_value(current_altitude)

        thruster_force = 0.2*pid_out + 0.8*last_thruster_force

        # we could check if the ellapsed time in CONTROL phase is higher than our mission time
        time_controlling = t - start_control_time
        if (time_controlling > mission_duration):
            print ("Starting SURFACING mode")
            print ("Time: ", t, "\tDepth: ", depth)
            mode = 'SURFACING'
            # Turn off the thruster!
            thruster_force = 0

    # ------------------------------------------------------------------------------------------------------
    # SURFACING MODE: passive surfacing procedure, where remaining ballast is dispensed (ws)
    elif mode == 'SURFACING':
        if (number_of_balls > 0) and (t - dropped_ball_time) > min_dispensing_time:
                #  reset dropped ball timestamp
                dropped_ball_time = t
                # drop a single ball
                number_of_balls -= 1

    error_history.append(altitude_error)
    acceleration_dive_history.append(acceleration)
    horizontal_position_history.append(horizontal_position)
    floor_profundity_history.append(floor_profundity)
    time_dive_history.append((time_step+t))
    velocity_dive_history.append(vertical_velocity)
    depth_dive_history.append(depth)
    altitude_history.append(current_altitude)
    net_buoyancy_dive_history.append(net_force)
    drag_dive_history.append(drag_force)
    thruster_history.append(thruster_force)
    balls_history.append(number_of_balls)
    t += time_step

print ("\nEnd of simulation. Depth: ", depth, " @ t: ", t)
####################################################################
# Ends current simulation, and start dumping data to the output file
output_df = pd.DataFrame(
    {'time': time_dive_history,
     'acceleration': acceleration_dive_history,
     'velocity': velocity_dive_history,
     'depth': depth_dive_history,
     'seafloor': floor_profundity_history,
     'altitude': altitude_history,
     'balls': balls_history,
     'x_position': horizontal_position_history,
     'thruster': thruster_history
    })

output_file = configuration['output']['file_path'] + configuration['output']['file_preffix'] + simulation_details + configuration['output']['file_extension']
output_file_html = configuration['output']['file_path'] + "plot" + simulation_details + ".html"

# WARNING: removed to speed up simulation process
output_df.to_csv(output_file, encoding='utf-8', index=False)

stop_time = time.time()
elapsed_time = stop_time - start_time
time.strftime("%H:%M:%S", time.gmtime(elapsed_time))

####################################################################
# Creates plots using Plotly
####################################################################
print ("Creating plots ...")

if configuration['output']['export_html'] == True:
    trace1 = go.Scatter(y=depth_dive_history, x=time_dive_history, name='Depth')
    trace2 = go.Scatter(y=velocity_dive_history, x=time_dive_history, name='Velocity')
    trace3 = go.Scatter(y=thruster_history, x=time_dive_history, name='Thruster')
    trace4 = go.Scatter(y=balls_history, x=time_dive_history, name='Balls')
    trace5 = go.Scatter(y=floor_profundity_history, x=time_dive_history, name='Profundity')
    trace6 = go.Scatter(y=error_history, x=time_dive_history, name='Altitude error')

    fig = tools.make_subplots(rows=2, cols=2, subplot_titles=('Depth', 'Velocity',
                                                              'Thruster', 'Balls'))
    fig.append_trace(trace1, 1, 1)
    fig.append_trace(trace5, 1, 1)

    fig.append_trace(trace2, 1, 2)

    fig.append_trace(trace3, 2, 1)
    fig.append_trace(trace6, 2, 1)

    fig.append_trace(trace4, 2, 2)

    fig['layout']['xaxis1'].update(title='Time (s)')
    fig['layout']['xaxis2'].update(title='Time (s)')
    fig['layout']['xaxis3'].update(title='Time (s)')
    fig['layout']['xaxis4'].update(title='Time (s)')

    fig['layout']['yaxis1'].update(title='Depth (m) & Profundity (m)', autorange='reversed')
    fig['layout']['yaxis2'].update(title='Velocity (m/s)')
    fig['layout']['yaxis3'].update(title='Thruster (N) & Altitude error (m)')
    fig['layout']['yaxis4'].update(title='Number of balls (#)')

    fig['layout'].update(title='Customizing Subplot Axes')

    print ("Done, now storing plots in ", output_file_html)
    print ("This may take a while ...")
    py.plot(fig, filename = output_file_html)
else:
    print ("Configuration set export_html = False")
    exit()
