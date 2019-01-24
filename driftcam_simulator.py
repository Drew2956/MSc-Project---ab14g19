# Driftcam diving simulator. 
# Created by: M. Massot
# Edited by: M. Massot, J. Cappelletto

import math
import pandas as pd
import plotly.offline as py
from plotly import tools
import plotly.graph_objs as go
from SimplePID import SimplePID

# TODO: repetitive linear searchs in the depth/density fields for the interpolation slow down the whole simulation. Perhaps is better 
# to perform a single fine grain interpolation, and then work with a down-to-cm resolution lookup table

# TODO: fix data headers for new testing data (from ctd_seawater_density calculation scripts) as they included the units in the variable name

# As realease v1.0 get_interpolated function is not being used beacuse of the high computational cost during list search
# This could be replaced (for dev purposes) by a polynomial curve or use the ordered list with a binomial search algorithm
def get_interpolated(df, field1, search_value, field2):
    # Initialize the variables
    anterior = 0
    posterior = 0
    index_anterior = 0
    index_posterior = 1
    not_found = True
    # While the values have not been found and we have not reached the end of the list
    while not_found and index_posterior < len(df[field1]):
        anterior = df[field1][index_anterior]
        posterior = df[field1][index_posterior]
        if anterior < search_value and posterior >= search_value:
            not_found = False
        elif anterior > search_value and posterior <= search_value:
            not_found = False
        else:
            index_anterior += 1
            index_posterior += 1
    if not not_found:
        # Prepare to interpolate our desired value
        x = search_value
        x1 = df[field1][index_anterior]
        x2 = df[field1][index_posterior]
        y1 = df[field2][index_anterior]
        y2 = df[field2][index_posterior]

        m = (y2-y1)/(x2-x1)
        return m*(x-x1)+y1
    else:
        print('Value {0} not found!'.format(search_value))


# read input data (density profile)
df = pd.read_csv('data/UDP-SB-CTD-RAW_20180801-161750_seawater_density.csv')

# Starting values for the simulation
depth = 40.0  # m
vertical_velocity = 0.0
t = 0.0

# Other simulation parameters 
time_interval = 0.1     # Euler solver time step. Ideal step values: 0.1 , as smaller steps doesn't provide any noticeable change in system dynamics response
min_dispensing_time = 7 # minimum admissible time between drop ball event
dropped_ball_time = 0   # timestamp of last dropped ball event

# variables you don't want to change unless you know what you're doing
# define constants andd platform dimensions
gravity = 9.81
drag_coefficient = 1.2
radius = 0.2
area = (math.pi*radius**2)
Tau = 7 # system dynamic response constant

# this can be loaded from an external YAML
main_mass = 37.5 # kg
main_volume = 0.03 # m3
ball_volume = 4/3*math.pi*math.pow(0.008,3)
ball_density = 7800 # kg / m3 # ball_mass = 0.016728 # kg 
ball_mass = ball_volume * ball_density
number_of_balls = 90 # initial number of balls

flotation_mass = 4.72 # kg
flotation_density = 400.0 # kg/m3
flotation_volume = flotation_mass / flotation_density

#  Empty placeholders for incoming simulation data
time_dive_history = []
velocity_dive_history = []
depth_dive_history = []
net_buoyancy_dive_history = []
drag_dive_history = []
balls_history = []
acceleration_dive_history = []

pid = SimplePID(590, -100, 100, 10, 0.1, 0.001 )

last_velocity = vertical_velocity

while depth < 800 and t < 5000:     # limit simulation time and depth (due to speed constraints)

    # replace weight with mass
    microballast_mass = ball_mass * number_of_balls # kg
    microballast_volume = ball_volume * number_of_balls # m3

    # complete system volume: platform + flotation + microballasts
    total_volume = flotation_volume + main_volume + microballast_volume
    total_mass = (main_mass + microballast_mass + flotation_mass)
    total_weight = total_mass*gravity

    ####################
    # See if it is possible to improve speed while doing the density search in the input vector (already ordered)
    seawater_density = 1029.6
    ball_buoyancy = ball_volume*gravity*seawater_density    # weight of the displaced seawater volume (per ball)
#    seawater_density = get_interpolated(df, 'Depth', depth, 'Density')

    buoyancy_force = total_volume*gravity*seawater_density
    drag_force = 0.5*drag_coefficient*seawater_density*area*abs(vertical_velocity)*vertical_velocity
    # calculate the actual net force experienced by the platform
    net_force = total_weight - buoyancy_force - drag_force
    # The real accelaration is obtained from F = m . a 
    acceleration = net_force / total_mass
    # The velocity is calculated via Euler integration for the acceleration 
    vertical_velocity = vertical_velocity + acceleration*time_interval
#    force_drag = total_weight - force_buoyancy  # assume it acelerates to terminal velocity
    depth = depth + vertical_velocity*time_interval

   # TODO: Discuss how to actually define the trigger 
    # It still depends on the seawater_density!!!
    future_velocity = vertical_velocity + 6*(vertical_velocity-last_velocity)

    new_drag = (main_mass + ball_mass * (number_of_balls-1) + flotation_mass)*gravity - ((flotation_volume + main_volume + ball_volume * (number_of_balls-1))*gravity*seawater_density)
    # drop a single ball
    if (depth > 500) and (number_of_balls > 0) and (t - dropped_ball_time) > min_dispensing_time and (future_velocity > 0):
#    if (depth > 500) and (output < -ball_buoyancy) and (number_of_balls > 1) and (t - dropped_ball_time) > min_dispensing_time and (new_drag > 1):
        dropped_ball_time = t
        number_of_balls -= 1
        last_velocity = vertical_velocity

    print('-------------------------------')
    print('Time {0}'.format(t))
    print('Density {0}'.format(seawater_density))
    print('# Balls {0}'.format(number_of_balls))
    print('Weight: {}'.format(total_weight))
    print('Buoyancy: {}'.format(buoyancy_force))
    print('Drag: {}'.format(drag_force))
    print('Net force: {}'.format(net_force))
    print('Acceleration: {}'.format(acceleration))
    print('Velocity: {}'.format(vertical_velocity))
    print('Depth: {}'.format(depth))
#    print('PID {0}'.format(pid.get_output_value(depth)))

    output = pid.get_output_value(depth)


    # to avoid non-valid cases    
    if depth < 0:
        break

    acceleration_dive_history.append(acceleration)
    time_dive_history.append((time_interval+t))
    velocity_dive_history.append(vertical_velocity)
    depth_dive_history.append(depth)
    net_buoyancy_dive_history.append(net_force)
    drag_dive_history.append(drag_force)
    balls_history.append(number_of_balls)
    t += time_interval

####################################################################
# Ends current simulation, and start dumping data to the output file
output_df = pd.DataFrame(
    {'time': time_dive_history,
     'acceleration': acceleration_dive_history,
     'velocity': velocity_dive_history,
     'depth': depth_dive_history,
     'drag': drag_dive_history,
     'balls': balls_history
    })

output_df.to_csv('driftcam_simulation.csv', encoding='utf-8', index=False)


####################################################################
# Creates plots using Plotly
trace1 = go.Scatter(y=depth_dive_history, x=time_dive_history, name='Depth')
trace2 = go.Scatter(y=velocity_dive_history, x=time_dive_history, name='Velocity')
trace3 = go.Scatter(y=acceleration_dive_history, x=time_dive_history, name='Acceleration')
trace4 = go.Scatter(y=balls_history, x=time_dive_history, name='Balls')

data = [trace1, trace2, trace3, trace4]

fig = tools.make_subplots(rows=2, cols=2, subplot_titles=('Depth', 'Velocity',
                                                          'Acceleration', 'Balls'))
fig.append_trace(trace1, 1, 1)
fig.append_trace(trace2, 1, 2)
fig.append_trace(trace3, 2, 1)
fig.append_trace(trace4, 2, 2)

fig['layout']['xaxis1'].update(title='Time (s)')
fig['layout']['xaxis2'].update(title='Time (s)')
fig['layout']['xaxis3'].update(title='Time (s)')
fig['layout']['xaxis4'].update(title='Time (s)')

fig['layout']['yaxis1'].update(title='Depth (m)', autorange='reversed')
fig['layout']['yaxis2'].update(title='Velocity (m/s)')
fig['layout']['yaxis3'].update(title='Acceleration (m/s2)')
fig['layout']['yaxis4'].update(title='Number of balls (#)')

fig['layout'].update(title='Customizing Subplot Axes')

py.plot(fig)