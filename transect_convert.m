% Import data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function transect_convert (filename, x_resolution)
	printf ("Loading %s ...", filename)
	tmp = load (filename);
	% Expected data format, from QGID 3.2 Profile plugin
	% X[km] LAT[°] LON[°] Depth[m]  
	printf ("done\n")
	% First, we trim the last row, as contains repeated value for X (discontinuity problem for interpolation)
	N = length(tmp(:,1)) - 4;
	tmp = tmp(1:N,:);

	% Then, we rearrange the data to a standard format:
	% X[km] Depth[m] LAT[°] LON[°] 
	x = tmp (:,1) * 112000;	% X position is given in 1:1000m
	y = -tmp (:,2);   % Depth is given as negative value

	plot (x,y)
	hold on

	% Now, we proceed to interpolate with the new resolution
	printf ("%d rows read from the input file\n", N)
	xmax = max (x)
	xf = [0:x_resolution:xmax]';
	yf = interp1 (x, y, xf, "pchip");	% spline tend to overshoot 
	% figure
	plot (xf, yf, 'r')

	N = length(xf(:,1));
	printf ("%d rows exported to the output file\n", N)
	printf ("Exporting interpolated data...\n")
	out(:,1) = xf;
	out(:,2) = yf;

	% Finally, we export the new data
	% TODO: trim extension
	dlmwrite(strcat (filename, "_interpolated.csv"), out, 'delimiter', '\t');
	
% Export data
