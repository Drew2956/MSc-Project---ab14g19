% Import data from summary table
data = dlmread ('../data/simulations/eta_test/Test01_DivingSurfacing_vs_ETA.xls','', 12, 1);
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


color1 = [0.8500    0.3250    0.0980]
color2 = [0    0.4470    0.7410]
%    0.9290    0.6940    0.1250
%    0.4940    0.1840    0.5560
%    0.4660    0.6740    0.1880
%    0.3010    0.7450    0.9330
%    0.6350    0.0780    0.1840

set (h(1), "facecolor", color1)
set (h(2), "facecolor", color2)

hold on
e1 = errorbar (eta-0.002, dive_avg, dive_std,'k--',1,1)
e2 = errorbar (eta+0.002, surf_avg, surf_std,'k--')

%title ('Diving and Surfacing time [min] vs \eta', "fontsize", 19)
xlabel ('Safety factor: \eta', "fontsize", 20)
ylabel ("Time (min)", "fontsize", 20)

lgd = legend ("Diving", "Surfacing")
set (lgd, "fontsize", 22)
set (lgd, "fontname", "arial")

ax = gca()
set (ax, "linewidth", 1)
set (ax, "fontname", "arial")
set (ax, "fontsize", 18)
