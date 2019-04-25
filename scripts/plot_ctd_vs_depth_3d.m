% Import data from summary table
function lgd = plot_ctd_vs_depth_3d(file_list, max_depth = 1000)
	close all
	% Look for all CSV files in the provided folder root (recursively)
	nfiles = size(file_list)(1);
	printf ("Total number of files: %d\n", nfiles)

	figure("position", [0,0,800,800])

	ax = gca()
	grid minor on

%	set (ax, "resize", "off")
	set (ax, "linewidth", 1)
	set (ax, "fontname", "arial")
	set (ax, "fontsize", 16)
	set (ax, "labelfontsizemultiplier", 1.2)
	set (ax, "minorgridlinestyle", ":")
	set (ax, "yminorgrid", "off")
	set (ax, "ygrid", "on")

	xlabel ("Density (kg/m3)")
	ylabel ("Depth (m)")
	zlabel ("CTD sample")
	title ("Density vs depth for 10 CTD profiles") 
	hold on
	for i = 1:nfiles
		% Need to trim a trailing whitespace in the end
		filename = strtrim(file_list(i,:));
		l = length(filename);
		filename = filename(1:l);

		[fPath fName fExtension] = fileparts (filename);
		printf ("Loading %s ...\n", fName)
		data = dlmread (filename, '');

		plot3 (data(1:max_depth,2), data(1:max_depth,1), i*ones(max_depth,1), 'linewidth', 3.5)
		density (:,i) = data(1:max_depth,2);

		% trim down to defined max_depth
	end

	[azim, elev] = view()
	view ([0 0.5 0.5])

%	lgd = legend (file_list(:,end-8:end-4))
%	set (lgd, "fontsize", 14)
%	set (lgd, "fontname", "arial")
%	set (lgd, "box", "on")
%	set (lgd, "location", "northeastoutside")
%	set (lgd, "position", [0.2 0.15 0.2 0.4])
	xlim ([1022 1032])

%	set(ax,'Ydir','reverse')

