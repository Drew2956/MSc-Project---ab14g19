% Receive path to CSV files containing the simuation output with the template:
% batch_e0.03_d0.014_cCTD01_tT1_power_mode.csv

% Current implementation includes downsampling capabiities, using an integer factor K
% It also includes a 'transect' flag for subgroup processing
function [power_history error_history] = plot_power_diameter(file_path, transect = 1)

	ctd_list = [1:10]
	ball_list = [0.014 : 0.002 : 0.020] 
	color_list = ['g' 'b' 'r' 'c']

	close all
	% Look for all CSV files in the provided folder root (recursively)
	K = 1;

	power_history = zeros(36001,4,10)
	error_history = zeros(36001,4,10)

	for id_ball = 1:4
		for ctd = 1:10
			ball = ball_list(id_ball)
			% batch_e0.03_d0.014_cCTD01_tT1_power_mode

			_filename = strcat ("batch_e0.03_d",sprintf("%.3f",ball),"_cCTD",sprintf("%02d",ctd),"_tT",num2str(transect)) 
	 		printf ("Processing @ process_transect_simulation[%s] ...\n", _filename)
	 		filename = strcat(file_path, _filename,"_power_mode.csv")
			data = dlmread (filename, '\t');

			% Deal with empty simulations (TWO simulations for T2 with d14mm failed)
			if (size(data) == 0)
				data = zeros (360001,5);
				data(1,4) = 2;
			endif

			data = data(1:K:end,:);

			_time = data (:,1);
			_x_pos = data (:,2);
			_power = data (:,3);
			_mode = data (:,4);
			_error = data (:,5);

			% find the first incidence of CONTROL mode
			start_control = find (_mode == 2, 1)

			power_history(:,id_ball,ctd) = _power;
			error_history(:,id_ball,ctd) = _error;

		    power_mean = mean(abs(_power(start_control:end)))
		    power_stdev = std (abs(_power(start_control:end)))

		    error_mean = mean(abs(_error(start_control:end)))
		    error_std = std(abs(_error(start_control:end)))
		    error_max = max(abs(_error(start_control:end)))

%			plot (_x_pos(start_control:K:end), _power(start_control:K:end), color_list(id_ball));
%			hold on
			newname = strcat(file_path,_filename,"_ds.csv") 
			dlmwrite(newname, data, 'delimiter', '\t');			
		end
	end



