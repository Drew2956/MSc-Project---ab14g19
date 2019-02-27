% Reads table with meand/std values of power and error for Transects: T1-T2-T3, Diameters: 14-16-18-20, CTD: 01-02-03...10

function [ax_list] = bar_power_diameter(filename)
	close all

	color = [0.8 0.5 0; 0.5 0 1; 0.5 0.8 0; 0 0.5 1];
	color = [0 0 1; 0 1 0; 1 1 0; 1 0 0];

	color = [0.8500    0.3250    0.0980; 0    0.4470    0.7410; 0.9290    0.6940    0.1250; 0.4940    0.1840    0.5560; 0.4660    0.6740    0.1880; 0.3010    0.7450    0.9330;  0.6350    0.0780    0.1840]


	transect_list = [1:3];
	ball_list = [0.014 : 0.002 : 0.020]; 
	ax_list = 0;
	% skip first row as it is the header
	data = dlmread (filename, '\t', 1, 0)
	% Expected data format:
	% Power_avg	Power_std	Error_avg	Error_std	Error_max	Transect	Diameter

	power_mean (1,1:4) = data(1:4,1);	% extract 1st transect, 1st column
	power_mean (2,1:4) = data(5:8,1);	% extract 2nd transect, 1st column
	power_mean (3,1:4) = data(9:12,1);	% extract 3rd transect, 1st column
	h = bar (transect_list, power_mean, 0.85)			% grouped barplot. Reverse index for diameters, so we get a nice small-to-big progression

	color (1,:)

	set (h(1), "facecolor", color(1,:))
	set (h(2), "facecolor", color(2,:))
	set (h(3), "facecolor", color(5,:))
	set (h(4), "facecolor", color(6,:))

	view (90, 90)					% rotate the view (axis swap)
	set (gca, "xgrid", "off")		% remove X grid as they are locations, not continuous variable
	%U+2300
	lgd = legend ('⌀ = 14 mm', '⌀ = 16 mm', '⌀ = 18 mm', '⌀ = 20 mm')	% add legend for diameters
	set (lgd, "fontsize", 20)
	%set (lgd, "fontname", "arial")
	set (lgd, "location", 'southeast')

	ax = gca();
	grid on
	set (gca, "xgrid", "off")	% we staty only with the vertical (now horizontal grid)
	%set (ax, "fontname", "arial")
	set (ax, "minorgridlinestyle", ":")
	set (ax, "yminorgrid", "off")
	set (ax, "fontsize", 18)
	set (ax, "labelfontsizemultiplier", 1.2)
	set (ax, "linewidth", 1)

	power_std (1, 1:4) = data(1:4,2);	% extract 1st transect, 2nd column
	power_std (2, 1:4) = data(5:8,2);	% extract 2nd transect, 2nd column
	power_std (3, 1:4) = data(9:12,2);	% extract 3rd transect, 2nd column

	hold on
	delta_x = [-0.3:0.2:0.3]
	%error_bars(power_mean, power_std)
	e1 = errorbar (delta_x + 1,  power_mean(1,:), power_std(1,:), 'k--')
	e2 = errorbar (delta_x + 2,  power_mean(2,:), power_std(2,:), 'k--')
	e3 = errorbar (delta_x + 3,  power_mean(3,:), power_std(3,:), 'k--')

	set (e1, 'linewidth', 1.0)
	set (e2, 'linewidth', 1.0)
	set (e3, 'linewidth', 1.0)

%	title ('Average power vs ball diameter for 3 transects', "fontsize", 19)
	xlabel ('Transect', "fontsize", 20)
	ylabel ("Thruster power (W)", "fontsize", 20)

	ax_list = [ax h(1) h(2) h(3) h(4) e1 e2 e3];
	return

