function gr_pts = find_animal(im,colors,cl_thresh)
%function uses color model and green location in color model to classify
%points as a shade of ear marker green or not. Pts returns empty if less
%than 2% of pixels are classified as green.

BOUNDS = false;

x=size(im,1);
y=size(im,2);
scale = 0.5;
bad_pts_thresh = 0; %0.1;



%help speed up computation, remove if undeeded for accuracy. speed up = x4
temp = imresize(im,scale);

temp = single(reshape(temp,scale.^2*x*y,3));

color_dist = pdist2(temp,colors); %%slooowwww, faster than knnsearch
[~,close_color] = min(color_dist,[],2);

gr_pts = close_color >= cl_thresh;

if any(gr_pts)
    gr_pts = reshape(gr_pts,scale.*x,scale.*y);
    
    if BOUNDS
        %don't include parts of image away from guinea pig head, or ass
        lower_bound = floor(200.*scale);
        upper_bound = ceil(540.*scale);
        gr_pts(:,1:lower_bound) = false;
        gr_pts(:,upper_bound:end) = false;
    end

    gr_pts = medfilt2(gr_pts,[3 3]);
    gr_pts = imresize(gr_pts,[x y]);
    
    percent_pts = sum(gr_pts(:))./numel(gr_pts)*100;
    if percent_pts <= bad_pts_thresh
        gr_pts = [];
    end
else
    gr_pts = [];
end





















