
% Perform the sectioning of walking data from the raw trajectories stored in c3d files
% It reads the c3d files from 'raw_data' folder and stores the mat files in the 'processed_data' folder of the 'A multi-sensor and cross-session human gait dataset'.

% ------- NEEDS TO DEFINE THE LOCAL PATH OF THE DATASET ----------
% Set the constant DATASET_PATH


% Dependencies:
%  MoCap toolbox: https://www.jyu.fi/hytk/fi/laitokset/mutku/en/research/materials/mocaptoolbox



load mcdemodata

DATASET_PATH = '';

users = dir(DATASET_PATH+'raw_data/');
users = users(~ismember({users(:).name},{'.','..'}));

for u=1:length(users)
	user = users(u).name;
	files_qtm = dir(DATASET_PATH+'raw_data/'+user+'/*_qtm.c3d');
	idxs = [];

	for i=1:length(files_qtm)
		capture_name = files_qtm(i).name;
		capture_data = mcread(strcat(files_qtm(1).folder, '/', capture_name))
		capture_qtm = mcinitstruct;
		capture_name = fieldnames(capture_data);
		
		idxs_init = [];
		idxs_end = [];
		markers_name = mcgetmarkername(capture_qtm);

		markers = {'tam_R', 'tam_L', 'fal_R', 'fal_L'};
		for k = 1:length(markers)
			mn = markers{k};
			idx_marker = find(strcmp(markers_name,mn)==1);
			marker_data = mcgetmarker(capture_qtm, idx_marker);
			x = marker_data.data(:, 1);
			fx = flip(x);
			idxs_init = [idxs_init find(x > (mean(x(1:100))+abs(0.1*mean(x(1:100)))), 1)];
			idxs_end = [idxs_end (length(x) - find(fx < (mean(fx(1:100))-abs(0.1*mean(fx(1:100)))), 1))];
		end

		idxs_init = sort(idxs_init);
		idxs_end = sort(idxs_end);

		if(abs(idxs_init(1) - idxs_init(4)) > 100)
			idx_init = max(idxs_init);
		else
			idx_init = floor(mean(idxs_init(1:2)));
		end

		if(abs(idxs_end(3) - idxs_end(4)) > 10)
			idx_end = min(idxs_end(3:4));
		else
			idx_end = ceil(mean(idxs_end(3:4)));
		end

		if(abs(idxs_end(1) - idxs_end(4)) > 100)
			idx_end = min(idxs_end);
		else
			idx_end = floor(mean(idxs_end(1:2)));
		end	

		idx_init = idx_init - 50;
		idx_end = idx_end + 50;
		capture_qtm_walk = mctrim(capture_qtm, idx_init, idx_end, 'frame');
		capture_qtm_walk.other.frame_init = idx_init;
		capture_qtm_walk.other.frame_end = idx_end;

		idxs = [idxs; [capture_name, capture_data.(capture_name{1}).other.frame_init, capture_data.(capture_name{1}).other.frame_end]];

		eval([ capture_name{1} ' = capture_qtm_walk']);
		save(DATASET_PATH+'processed_data/'+user+'/'+capture_name+'_qtm_walk.mat',  capture_name{1});
	end

	table_idxs = cell2table(idxs,'VariableNames',{'file', 'idx_init', 'idx_end'});
	writetable(table_idxs, DATASET_PATH+'raw_data/'+user+'/idxs_frames.csv');

end