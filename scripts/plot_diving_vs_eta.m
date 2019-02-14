% Import data from summary table
data = dlmread ('data/simulations/eta_test/Test01_DivingSurfacing_vs_ETA.xls','', 12, 1);
% Recreate ETA vector
eta = [0.01:0.01:0.05];

dive_avg = data(1,1:2:end)/60;
dive_std = data(2,1:2:end)/60;

surf_avg = data(1,2:2:end)/60;
surf_std = data(2,2:2:end)/60;

close all
figure("position", [0,0,800,800])

ax = gca()
grid on

set (ax, "fontname", "arial")
set (ax, "minorgridlinestyle", ":")
set (ax, "yminorgrid", "off")
set (ax, "xgrid", "off")
set (ax, "fontsize", 16)
set (ax, "labelfontsizemultiplier", 1.2)

h = bar (eta, [dive_avg' surf_avg'], .9)

set (h(1), "facecolor", "r")
set (h(2), "facecolor", "g")

hold on
e1 = errorbar (eta-0.002, dive_avg, dive_std,'k--',1,1)
e2 = errorbar (eta+0.002, surf_avg, surf_std,'k--')

title ('Diving and Surfacing time [min] vs \eta', "fontsize", 19)
xlabel ('\eta', "fontsize", 18)
ylabel ("Time (m)", "fontsize", 18)

lgd = legend ("Diving", "Surfacing")
set (lgd, "fontsize", 20)
set (lgd, "fontname", "arial")

ax = gca()
set (ax, "linewidth", 1)
set (ax, "fontname", "arial")
set (ax, "fontsize", 16)
