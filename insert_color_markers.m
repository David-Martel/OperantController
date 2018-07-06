function im_new = insert_color_markers(im,gr_pts)

x = size(im,1);
y = size(im,2);
im_new = reshape(im,x*y,3);
green_idx = reshape(gr_pts,x*y,1);

im_new(green_idx,:) = repmat(uint8([0 255 0]),sum(green_idx),1);
im_new = reshape(im_new,x,y,3);













