import math
import pandas as pd
import plotly.offline as py
from plotly import tools
import plotly.graph_objs as go
from SimplePID import SimplePID


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



df = pd.read_csv('UDP-SB-CTD-RAW_20180801-161750_seawater_density.csv')

depth = 500.0  # m
main_weight = 37.5 # kg
main_volume = 0.03 # m3
ball_volume = 4/3*math.pi*math.pow(0.008,3)
ball_weight = 0.016728 # kg
number_of_balls = 90 # number of balls

flotation_weight = 4.72 # kg
flotation_density = 400.0 # kg/m3
flotation_volume = flotation_weight / flotation_density

time_interval = 0.5
t = 0

# variables you don't want to change unless you know what you're doing
gravity = 9.81
drag_coefficient = 1.2
radius = 0.2
area = (math.pi*radius**2)

time_dive_history = []
velocity_dive_history = []
depth_dive_history = []
net_buoyancy_dive_history = []
drag_dive_history = []
balls_history = []

dropped_ball_time = 0
pid = SimplePID(590, -100, 100, 10, 0.1, 0.001 )


while depth < 600 and t < 1000:
    microballast_weight = ball_weight * number_of_balls # kg
    microballast_volume = ball_volume * number_of_balls # m3

    volume_total = flotation_volume + main_volume + microballast_volume
    weight_total = (main_weight + microballast_weight + flotation_weight)*gravity

    seawater_density = get_interpolated(df, 'Depth', depth, 'Density')
    buoyancy = volume_total*gravity*seawater_density
    drag = weight_total - buoyancy  # assume it acelerates to terminal velocity

    ball_buoyancy = ball_volume*gravity*seawater_density

    new_drag = (main_weight + ball_weight * (number_of_balls - 1) + flotation_weight)*gravity - ((flotation_volume + main_volume + ball_volume * (number_of_balls - 1))*gravity*seawater_density)

    print('----------------------------------')
    print('Time {0}'.format(t))
    print('Weight: {}'.format(weight_total))
    print('Buoyancy: {}'.format(buoyancy))
    print('Drag: {}'.format(drag))
    print('Depth: {}'.format(depth))
    print('PID {0}'.format(pid.get_output_value(depth)))
    print('# Balls {0}'.format(number_of_balls))
    print('Density {0}'.format(seawater_density))

    output = pid.get_output_value(depth)
    if output < -ball_buoyancy and number_of_balls > 0 and t - dropped_ball_time > 5.0 and new_drag > 0:
        dropped_ball_time = t
        number_of_balls -= 1

    if drag > 0:
        vertical_velocity = math.pow(drag/(drag_coefficient*seawater_density*(area/2)), 0.5)
    else:
        vertical_velocity = -math.pow(-drag/(drag_coefficient*seawater_density*(area/2)), 0.5)

    depth = vertical_velocity*time_interval+depth
    
    if depth < 1.5:
        break

    time_dive_history.append((time_interval+t))
    velocity_dive_history.append(vertical_velocity)
    depth_dive_history.append(depth)
    net_buoyancy_dive_history.append((weight_total-buoyancy)/9.81)
    drag_dive_history.append(drag)
    balls_history.append(number_of_balls)
    t += time_interval

output_df = pd.DataFrame(
    {'time': time_dive_history,
     'velocity': velocity_dive_history,
     'depth': depth_dive_history,
     'drag': drag_dive_history,
     'balls': balls_history
    })

output_df.to_csv('driftcam_simulation.csv', encoding='utf-8', index=False)

trace1 = go.Scatter(y=depth_dive_history, x=time_dive_history, name='Depth')
trace2 = go.Scatter(y=velocity_dive_history, x=time_dive_history, name='Velocity')
trace3 = go.Scatter(y=drag_dive_history, x=time_dive_history, name='Drag')
trace4 = go.Scatter(y=balls_history, x=time_dive_history, name='Balls')

data = [trace1, trace2, trace3, trace4]


fig = tools.make_subplots(rows=2, cols=2, subplot_titles=('Depth', 'Velocity',
                                                          'Drag', 'Balls'))
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
fig['layout']['yaxis3'].update(title='Drag force (N)')
fig['layout']['yaxis4'].update(title='Number of balls (#)')

fig['layout'].update(title='Customizing Subplot Axes')

py.plot(fig)    