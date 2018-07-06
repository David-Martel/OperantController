function im = double2uint8(im)

im = im./max(abs(im(:)));
im = uint8(255*im);






