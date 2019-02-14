% Import data from summary table
function lgd = plot_transects(file_list, max_length = 2000)
	close all
	% Look for all CSV files in the provided folder root (recursively)
	nfiles = size(file_list)(1);
	printf ("Total number of files: %d\n", nfiles)

	figure("position", [0,0,800,800])

	ax = gca()
	grid on

%	set (ax, "resize", "off")
	set (ax, "linewidth", 1)
	set (ax, "fontname", "arial")
	set (ax, "fontsize", 16)
	set (ax, "labelfontsizemultiplier", 1.2)
	set (ax, "minorgridlinestyle", ":")
	set (ax, "yminorgrid", "off")
	set (ax, "ygrid", "on")

	xlabel ("Distance (m)")
	ylabel ("Depth (m)")
	title ("Transect profiles @ Hydrate Ridge [-125.15°W, 44.57°N]") 
	hold on

	K = 10;
	M = floor(K*max_length)
	for i = 1:nfiles
		% Need to trim a trailing whitespace in the end
		filename = strtrim(file_list(i,:));
		l = length(filename);
		filename = filename(1:l);

		[fPath fName fExtension] = fileparts (filename);
		printf ("Loading %s ...\n", fName)
		data = dlmread (filename, '');
		data_s = data(1:K:end,:); 

		% trim down to defined max_depth
		plot (data_s(1:M,1),data_s(1:M,2), 'linewidth', 1.5)
		str_lgd(i,:) =  strcat ("Transect 0", int2str(i));
	end

	lgd = legend (str_lgd)
	set (lgd, "fontsize", 18)
	set (lgd, "fontname", "arial")
	set (lgd, "box", "on")
	set (lgd, "position", [0.68 0.15 0.21 0.25])

	set(ax,'Ydir','reverse')

