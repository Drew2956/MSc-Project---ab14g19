# Driftcam diving simulator. 
# Created by: M. Massot
# Edited by: M. Massot, J. Cappelletto

# TODO: check which libraries are not being used 
import time
import math
import pandas as pd
import sys
import yaml
import argparse
import plotly.offline as py
from plotly import tools
import plotly.graph_objs as go
from SimplePID import SimplePID

verbose_flag = False

#TODO: improve output message and usage example
parser = argparse.ArgumentParser(

    description='Driftcam diving simulator. Uses CTD profiles, terrain model, ballast size and thruster model to recreate a diving/braking/mapping/surfacing sequence',
    usage='''driftcam_simulator.py <command> [<args>]''',
    formatter_class=argparse.RawDescriptionHelpFormatter)

# parser.add_argument('parse', help="Parse RAW data and convert it to an intermediate dataformat")
parser.add_argument("--ballast", help="Specify desired ballast diameter [mm]. This value overrides any configuration.yaml value")
parser.add_argument("--ctd", help="Specify the path to the CTD profile to be used for the simulation")
parser.add_argument("--transect", help="Path to seafloor depth profile")
parser.add_argument("--config", help="Specify YAML configuration file to be used")
parser.add_argument("--output", help="Define output file name")
parser.add_argument("--verbose", help="Ask for a more verbose output", action="store_true")
 
# Parsing command line arguments
args = parser.parse_args()

# Check if verbose flag was activated
if args.verbose:
    verbose_flag = True
    print ("Verbose mode ON")

#########################################

print ("**Driftcam_ diving simulator**")
print ("Loading configuration.yaml...")
config_file = "config/configuration.yaml"

start_time = time.time()

########################################
# Read configuration parameters from configuration.yaml
with open(config_file,'r') as stream:
    configuration = yaml.load(stream)   

########################################
# CTD PROFILE
########################################
if args.ctd:

    print ("CTD profile specified at runtime")
    # read input data (density profile)
    configuration['density_profile'] = args.ctd

print ("Loading CTD profile from: ", configuration['density_profile'])
# read input data (density profile)
density_table = pd.read_csv(configuration['density_profile'], sep='\t', header=None)

# This file must an already interpolated dataset, at 1.0 m depth resolution for faster calculations
# By doing this, the interpolation is done via lookup table, starting from 0 meter index

# Starting values for the simulation
depth = configuration['input']['start_depth']  # initial simulation depth
vertical_velocity = configuration['input']['start_velocity']   # initial vertical velocity. For faithfull simulations, it should be close to 0.0
horizontal_position = configuration['transect']['horizontal_position_offset']   # starting point in X axis for the transect mode simulations
t = 0.0
dropped_ball_time = 0   # timestamp of last dropped ball event


# Use the first value of the lookup table. In fixed seawater simulations, this will be the constant value to be used during all the simulation
seawater_density = density_table[1][int (math.floor(depth))]
# safety factor (eta) value 
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


# if ballast diameter is specified at launch time. It superseeds the value imported from configuration.yaml
if args.ballast:
    configuration['input']['ball_diameter'] = float(args.ballast)
    print ("Using ball_diameter defined at runtime: ", configuration['input']['ball_diameter'])

ball_diameter = configuration['input']['ball_diameter'] # ballast unit diameter
ball_volume = 4/3*math.pi*math.pow(ball_diameter/2,3)   # ballast unit volume
ball_density = configuration['input']['ball_density'] # kg / m3 # ball_mass = 0.016728 # kg 
ball_mass = ball_volume * ball_density  # ballast unit mass

flotation_mass = configuration['input']['flotation_mass'] # kg
flotation_density = configuration['input']['flotation_density'] # kg/m3
flotation_volume = flotation_mass / flotation_density   # total volume of the flotation mass (assumed to be constant)

ballast_mass = configuration['input']['ballast_mass']   # design defined initial ballast_mass
number_of_balls = math.ceil(ballast_mass / ball_mass)   # rounded-up number of ballast units based on the desired ballast_mass and the ballas unit mass


braking_altitude = configuration['control']['braking_altitude']	# altitude threshold that will trigger change of platform behaviour from DIVING to BRAKING
mission_duration = configuration['control']['mission_duration'] # time duration of mission phase (active ALTITUDE CONTROL mode)

########################################
# TRANSECT DEFINITION
########################################

# if 'transect' is specified at launch moment, the load it
if args.transect:
    print ("Seafloor model specified at runtime: ", args.transect)
    floor_model = 'transect'
    configuration['transect']['transect_profile'] = args.transect
    profundity_table = pd.read_csv(configuration['transect']['transect_profile'], sep='\t', header=None)
    floor_model = 'transect'
    floor_profundity = profundity_table[1][0]
    floor_id = configuration['transect']['transect_profile'].split(".")[0]
# else, check the floor_model defined in the configuration.yaml
else:
    floor_model = configuration['floor_model']  # retrieve floor model: fixed or transect
    if floor_model == 'fixed':
        print ("Using 'floor_model': ", floor_model)
        floor_profundity = configuration['control']['floor_profundity']
        floor_id = str(floor_profundity)
    elif floor_model == 'transect':
        print ("Importing transect profile from: ", configuration['transect']['transect_profile'])
        profundity_table = pd.read_csv(configuration['transect']['transect_profile'], sep='\t', header=None)
        floor_profundity = profundity_table[1][0] + configuration['transect']['transect_profundity_offset']
        floor_id = configuration['transect']['transect_profile'].split("_")[0]
        floor_id = floor_id.split("/")[-1]
    else:
        print ("Unknown 'floor_model' ", floor_model, " defined in 'configuration.yaml'")
        print ("Using default 'fixed' depth model with depth = 100m")
        floor_model = 'fixed'
        floor_profundity = 100

# intermediate values for simple low pass filter of the floor profundity. This is done to avoid step-like response from the controller
_t1_floor_profundity = floor_profundity
_t0_floor_profundity = floor_profundity

########################################
# Print summary of simulation parameters		TODO: improve summary table
########################################
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

if args.output:	# if 'output' flag was defined, then load the desired output filename
    print ("Runtime defined output filename: ", args.output)
    simulation_details = args.output
else:	# if no explicit output was defined, then we create a fully identified string to be used as output filename
    # string used for the output file name (both CSV and HTML outputs)
    density_id = configuration['density_profile'].split("_")[1]
    density_id = density_id.split("/")[-1]
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
mode = 'DIVING'	# mode variable that will reflect accordingly the behaviour of the platform

thruster_force = 0  # starting with the thrusters OFF
safety_factor_altitude = 1.0    # safety margin to trigger phase transition from BRAKING to CONTROL (basically increases the Htarget for the control, so it fires the thrusters before reaching the target)
target_altitude = configuration['control']['target_altitude']   # reference value for the altitude controller

# Loading PID contrroller gains from the configuration
_kp = configuration['control']['kp_gain']
_ki = configuration['control']['ki_gain']
_kd = configuration['control']['kd_gain']

# creates SimplePID controller object, with defined gains and hard limits for the control output
pid = SimplePID(target_altitude, -100, 100, _kp, _ki, _kd, time_step * 1000 )

# execution flag, to be employed in the main simulation loop
keep_running = True

# Stop conditions for simulation: total simulation time, and any specific criteria defined in the body of the loop (in those cases, use keep_running flag)
while (t < simulation_time) and (keep_running == True):     # limit simulation time and depth (due to speed constraints)

	# we detect if the altitude is negative (plus an additional gap, currently of 10 meters)
    if (depth > (floor_profundity + 10)):
        print ("\n>>>>>>>>>\tPlatform crashed!!!")
        print ("\n>>>>>>>>>\tHalting simulation...")
        print (floor_profundity)
        print (depth)
        keep_running = False	# triggers end of current simulation loop
        break
    ################################################
    # HORIZONTAL MOVEMENT SIMULATION
    # Only required for floor_model = 'transect' mode
    ################################################
    # current implementation simulates a simple constant speed movement along the seabed
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

        # cheap low-pass filter included to reduce the step-like behaviour of the bathymetric data empplyed in current simulations
        # probably is worth having a pre-filtered seafloor profile data
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
    # to avoid non-valid cases, negative depths (platform surfaced)
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

        ## low pass filter for the thruster force, implemented as suggested during tests for UT19 paper
        thruster_force = 0.2*pid_out + 0.8*last_thruster_force

        # we could check if the ellapsed time in CONTROL phase is higher than our mission time
        time_controlling = t - start_control_time
        if (time_controlling > mission_duration):	# did we exceed the mission duration? switch to SURFACING mode!
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

# construct the output file name based on the defined preffix, simulation parameters, and file extension
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

# for faster simulations, we can define that we don't want to generate the output HTML plots.
# this is the suggested setup for batch simulations
if configuration['output']['export_html'] == True:
    print ("Creating plots ...")
    trace1 = go.Scatter(y=depth_dive_history, x=time_dive_history, name='Depth')
    trace2 = go.Scatter(y=velocity_dive_history, x=time_dive_history, name='Velocity')
    trace3 = go.Scatter(y=thruster_history, x=time_dive_history, name='Thruster')
    trace4 = go.Scatter(y=balls_history, x=time_dive_history, name='Balls')
    trace5 = go.Scatter(y=floor_profundity_history, x=time_dive_history, name='Profundity')
    trace6 = go.Scatter(y=error_history, x=time_dive_history, name='Altitude error')

    fig = tools.make_subplots(rows=2, cols=2, subplot_titles=('Depth', 'Velocity',
                                                              'Thruster', 'Balls'))
    # depth and profundity
    fig.append_trace(trace1, 1, 1)
    fig.append_trace(trace5, 1, 1)
    # velocity
    fig.append_trace(trace2, 1, 2)
    # thruster and altitude error
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
