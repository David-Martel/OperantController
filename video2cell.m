function cell_array = video2cell(video)

num_pts = size(video,4);
cell_array = cell(num_pts,1);

for idx = 1:num_pts
    cell_array{idx} = video(:,:,:,idx);
end






