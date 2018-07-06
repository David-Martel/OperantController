clear
clc

close all


%% Get camera
alt_lab = false;

imaqreset;
if ~alt_lab
    vid = videoinput('pointgrey', 1, 'F7_BayerRG8_752x480_Mode0');
    src = getselectedsource(vid);
    src.WhiteBalanceRBMode = 'manual';
    src.WhiteBalanceRB = [419 719];
    src.BrightnessMode = 'Manual';
    src.Brightness = 186;
    src.Gain = 0.527;
    src.Exposure = 33;
    src.Gamma = 0;
    src.FrameRateMode = 'manual';
    src.FrameRate = 30;
else
    vid = videoinput('winvideo', 1, 'RGB24_1280x960');
    src = getselectedsource(vid);
    
    vid = videoinput('winvideo', 1, 'RGB24_1280x960');
src = getselectedsource(vid);

vid.FramesPerTrigger = 1;

triggerconfig(vid, 'manual');

% TriggerRepeat is zero based and is always one
% less than the number of triggers.
vid.TriggerRepeat = Inf;



%start(vid);



    
    
    
    
end
%configure camera parameters



flushdata(vid);
vid.FramesPerTrigger = inf;
drawnow;

% start(vid);


%% configure color model
%default model looks for green
% Base_Dir = pwd;
% load(fullfile(Base_Dir,'color_model_main.mat'),...
%     'color_model','gr_mdl','num_clust','num_shade');
%
% colors = [[210 240 210];[220 250 220]; color_model;gr_mdl]; %doesn't do so hot with bright white
% color_labels = (num_clust+(1:num_shade));
% cl_thresh = min(color_labels); %indicates in label matrix where GP colors begin

figure(1)
clf(1)
set(gca,'xtick',[]);
set(gca,'xticklabel',{});
set(gca,'ytick',[]);
set(gca,'yticklabel',{});
drawnow;

animal_name = 'test';

base_dir = pwd;
color_dir = 'Color_Data';
color_save = fullfile(base_dir,color_dir);
if ~exist(color_save,'dir')
    mkdir(color_save);
end

color_model_file = fullfile(color_save,sprintf('color_model_%s*.mat',animal_name));
file_list = dir(color_model_file);
if ~isempty(file_list)
    
    model = load(fullfile(color_save,file_list(1).name),'color_model','cl_thresh');
    color_model = single(model.color_model);
    cl_thresh = single(model.cl_thresh);
    TRAIN_MODEL = false;
else
    TRAIN_MODEL = true;
    color_model = [];
    cl_thresh = [];
end

SAVE_VIDS = false;

start_time = tic;
tot_frames = 1000;
control_var = true;
frame_iter = 1;
while control_var
    
    %% acquire new data for training
    if TRAIN_MODEL
        %% run tracker
         start(vid);
%         
         drawnow;

%         flushdata(vid);
%         drawnow;
        if alt_lab           
            trigger(vid);        
            pause(1.5);
            flushdata(vid);
            drawnow;
        end
%         
        while ~vid.FramesAvailable
        
        end
        %get image
        im = getdata(vid);
        im = im(:,:,:,end);
        %im = imresize(im,0.5);
        stop(vid);
        imshow(im);
        drawnow;
        
        uiwait(msgbox('Click on background'));
        [x_back,y_back] = getpts(figure(1));
        
        back_model = [];
        for idx = 1:length(x_back)
            back_model(idx,:) = im(floor(y_back(idx)),floor(x_back(idx)),:);
        end
        
        uiwait(msgbox('Click on guinea pig colors'));
        [x_gp,y_gp] = getpts(figure(1));
        
        gp_model = [];
        for idx = 1:length(x_gp)
            gp_model(idx,:) = im(floor(y_gp(idx)),floor(x_gp(idx)),:);
        end
        
        %model parameters
        color_model = [back_model; gp_model];
        cl_thresh = min(length(x_back)+(1:length(x_gp)));
        
        %find color
        gp_pts = find_animal(im,color_model,cl_thresh);
        
        %% recolor image
        im_marked = insert_color_markers(im,gp_pts);
        
        [xgp,ygp] = find(gp_pts==1);
        im_marked = insertMarker(im_marked,median([ygp xgp]),...
            'Marker','x-mark','Color','r');
        clf(1);
        
        imshow(im_marked);
        drawnow;
        
        resp = inputdlg('Enter "bad" to retry','Track accuracy',[1 35],{'bad'});
        if cellfun(@(x)~contains(lower(x),'bad'),resp,'uniformoutput',true)
            control_var = false;
            color_model_file = strrep(color_model_file,'*',datestr(now,'_mmm-dd-yyyy'));
            save(color_model_file,'color_model','cl_thresh');
        end
        
    else
        %% run tracker
        if ~isrunning(vid)
            start(vid);
            flushdata(vid);
        end
        if alt_lab
            trigger(vid);
            pause(1.5)
            flushdata(vid);
            drawnow;
        end
        
        %flushdata(vid);
        %get image
        while ~vid.FramesAvailable

        end
        im = getdata(vid,1);
        im = im(:,:,:,end);
        
        im = imresize(im,0.5);
        frame_iter = frame_iter + 1;
        
        %find color
        gp_pts = find_animal(im,color_model,cl_thresh);
        
        %% recolor image
        im = insert_color_markers(im,gp_pts);
        
        [xgp,ygp] = find(gp_pts==1);
        %get rid of outliers from centroid?
        im = insertMarker(im,median([ygp xgp]),...
            'Marker','x-mark','Color','r');
        clf(1);
        
        imshow(im);
        drawnow;
        if frame_iter >= tot_frames
            control_var = false;
        end
        
    end
    
    
    
    
end

if isrunning(vid)
    stop(vid);
end







