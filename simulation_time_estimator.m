% Import data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function simulation_time_estimator (file_list)

	% Look for all CSV files in the provided folder root (recursively)
	nfiles = size(file_list)(1);
	printf ("Total number of files: %d\n", nfiles)

	for i = 1:nfiles
		% Need to trim a trailing whitespace in the end
		filename = strtrim(file_list(i,:));
		l = length(filename);
		filename = filename(1:l);

		[fPath fName fExtension] = fileparts (filename);
		printf ("Loading %s ...", fName)
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

		diving_time(i) = time(idx);
		surfacing_time(i) = time(N) - diving_time(i);
		printf ("Diving time: %f\n", diving_time(i))
		printf ("Surfacing time: %f\n", surfacing_time(i))
		printf ("\t Total time: %f\n", time(N))
	
	endfor

	new_filename = strcat(fPath,"/","diving_time_list.csv")
    out = [diving_time' surfacing_time']
	dlmwrite(new_filename, out, 'delimiter', '\t');

