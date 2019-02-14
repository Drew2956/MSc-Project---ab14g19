% Import data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function transect_convert (filename, x_resolution, x_scale, y_scale = -1)
	printf ("Loading %s ...", filename)
	tmp = load (filename);
	% Expected data format, from QGID 3.2 Profile plugin
	% X[km] Depth[m]  Current Falkor data (and probably all incoming data uses negative depth values 
	printf ("done\n")
	% First, we trim the last row, as contains repeated value for X (discontinuity problem for interpolation)
	N = length(tmp(:,1)) - 1;
	tmp = tmp(1:N,:);

	% Then, we rearrange, scale and invert the data (if required) to our standard format:
	% X[km] Depth[m]
	xi = x_scale * tmp (:,1) ;  % X position is given in 1:100.000
	yi = y_scale * tmp (:,2);   % Depth is given as negative value. Using default y_scale = -1 will invert depth to positive

	plot (xi,yi)
	hold on

	% Now, we proceed to interpolate with the new resolution
	printf ("%d rows read from the input file\n", N)
	xmax = max (xi)
	xf = [0:x_resolution:xmax]';
	yf = interp1 (xi, yi, xf, "spline");	% spline tend to overshoot. Alternative are pchip and linear 
	% figure
	plot (xf, yf, 'r')

	N = length(xf(:,1));
	printf ("%d rows exported to the output file\n", N)
	printf ("Exporting interpolated data...\n")
	out(:,1) = xf;
	out(:,2) = yf;

	% Finally, we export the new data
	[fPath fName fExtension] = fileparts (filename)
	new_filename = strcat(fPath,"/",fName,"_interpolated.csv")

	dlmwrite(new_filename, out, 'delimiter', '\t');

