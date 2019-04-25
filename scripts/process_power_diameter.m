% Import data from summary table

function data = process_power_diameter(file_list)

	close all
	% Look for all CSV files in the provided folder root (recursively)
	nfiles = size(file_list)(1);
	printf ("Total number of files: %d\n", nfiles)

	% Faster if we preallocate space for the vectors
	power_mean = zeros(nfiles,1);
	power_std = zeros(nfiles,1);
	error_mean = zeros(nfiles,1);
	error_std = zeros(nfiles,1);
	error_max = zeros(nfiles,1);
	ctd_id = zeros(nfiles,1);
	ball_id = zeros(nfiles,1);
	tran_id = zeros(nfiles,1);

	for i = 1:nfiles

		% Need to trim a trailing whitespace in the end
		filename = strtrim(file_list(i,:));
		l = length(filename);
		filename = filename(1:l);

		[fPath fName fExtension] = fileparts (filename);
		printf ("Processing @ process_transect_simulation[%s] ...\n", fName)

		[pm ps em es emx _ctd _ball _trans] = process_transect_simulation (filename, 0, 0);
		power_mean(i) = pm;
		power_std(i) = ps;
		error_mean(i) = em;
		error_std(i) = es;
		error_max(i) = emx;
		ctd_id(i) = _ctd;
		ball_id(i) = _ball;
		tran_id(i) = _trans;
	end

	data = [power_mean power_std error_mean error_std ctd_id ball_id tran_id];
	dlmwrite('power_error_table.csv', data, 'delimiter', '\t');
	