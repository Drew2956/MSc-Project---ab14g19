function batch_list(list)
	for i=1:size(list)
		[a b c d e] = process_transect_simulation(list(i,:),0,0);
		data(i,1) = a;
		data(i,2) = b;
		data(i,3) = c;
		data(i,4) = d;
		data(i,5) = e;
	endfor

	dlmwrite("test02_results.txt", data, 'delimiter', '\t');

	
