% Import data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function simulation_time_estimator (filename)
	printf ("Loading %s ...", filename)
	data = dlmread (filename, ',', 1,0);
	% Expected data format, from QGID 3.2 Profile plugin
	% X[km] Depth[m]  Current Falkor data (and probably all incoming data uses negative depth values 
	printf ("done\n")
	% First, we trim the last row, as contains repeated value for X (discontinuity problem for interpolation)
	N = length(data(:,1));

	time = data(:,7);
	depth = data(:,4);
	thruster = data(:,6);

	[val idx] = max (thruster);

	diving_time = time(idx);
	surfacing_time = time(N) - diving_time;
	printf ("Diving time: %f\n", diving_time)
	printf ("Surfacing time: %f\n", surfacing_time)
	printf ("\t Total time: %f\n", time(N))

%	new_filename = strcat(fPath,"/",fName,"_interpolated.csv")

%	dlmwrite(new_filename, out, 'delimiter', '\t');

