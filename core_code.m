clear
clc

close all

figure(1);
clf(1);

SMALL_IMAGE = true;

cur_dir = pwd;
data_dir = fullfile(cur_dir,'DataStore');
addpath(genpath(pwd));

%% Get protocol information
animal_name = input('Input animal name','s');

run_time = datestr(now,'mmm-dd-yyyy-HH-MM-SS');
animal_name = [animal_name '_' run_time];

%use animal name to enable silence trials
if contains(lower(animal_name),'silence')
    ENABLE_SILENCE = true;
else
    ENABLE_SILENCE = false;
end

if contains(lower(animal_name),'retrain')
    NEW_COLOR = true;
else
    NEW_COLOR = false;
end

if contains(lower(animal_name),'email:')
    SEND_EMAIL = true;
    string_parts = strsplit(lower(animal_name),'_');
    email_address = '';
    for part_idx = 1:length(string_parts)
        
        part = string_parts{part_idx};
        if contains(part,'email:')
            email_address = strrep(part,'email:','');
        end
        
    end
    
    if isempty(email_address)
        disp('No email address entered; using default')
        email_address = 'the-shore-lab@umich.edu';
    end
    
    animal_name = strrep(animal_name,['_email:' email_address],'');
    
end

animal_folder = [data_dir filesep animal_name];
if ~exist(animal_folder,'dir')
    mkdir(animal_folder);
end

date_str = datestr(now,'mmm_dd_yyyy');
cur_time = datestr(now,'HH:MM:SS');

if ~exist([animal_folder filesep date_str],'dir')
    mkdir([animal_folder filesep date_str]);
end


save_file = [animal_folder filesep animal_name '_operant_info.mat'];
if ~exist(save_file,'file')
    animal_info = struct;
else
    data = load(save_file);
    animal_info = data.animal_info;
end

disp('Holding period ends at ')

%% Color clustering analysis -- test
train_time = tic;
if ~exist('color_model.mat','file') || NEW_COLOR
    disp('training color model');
    empty_frame_data = load('auto_stim_box.mat','frame2'); %load('empty_box','frame','frame2');
    temp_frame2 = empty_frame_data.frame2;
    
    [X,Y,COLOR] = size(temp_frame2);
    
    temp_frame2 = double(temp_frame2);
    temp_frame2 = temp_frame2-min(temp_frame2(:));
    temp_frame2 = temp_frame2./max(abs(temp_frame2(:)));
    
    temp_frame2 = rgb2hsv(temp_frame2);
    tf2 = reshape(temp_frame2,X*Y,COLOR);
    
    %learn colors of  background through Gaussian Mixture Model
    test_clusts = 20;
    %[idx,centroids] = kmeans(tf2,test_clusts,'MaxIter',400);
    gm_model = fitgmdist(tf2,test_clusts,'RegularizationValue',0.001,'Options',statset('MaxIter',400));
    color_model = gm_model.mu;
    save('color_model.mat','color_model')
else
    disp('loaded color model')
    load('color_model.mat','color_model');
end

color_thresh = 95;
training_time = toc(train_time);
%% 10 minute holding period, reflecting training time loss
if ~contains(lower(animal_name),'test')
    pause(60*10-training_time);
end

%% Get stim board
disp('Getting EStim board')
board_time = tic;
if ~exist('device','var')
    try
        device = arduino;
        device_found = true;
    catch
        device = [];
        device_found = false;
    end
end

if device_found
    
    %Disable current delivery pin
    [~, name] = system('hostname');
    name(end) = '';
    
    if ismember(name,{'khri-5434dmdesk','KHRI-305971'})
        current_pin = 'D6';
    else
        current_pin = 'D5';
    end
    
    writeDigitalPin(device,current_pin,0)
    
    %Configure SPI bus on arduino
    spi_bus = spidev(device,'D10');
    spi_bus.BitRate = 10000000;
    spi_bus.BitOrder = 'msbfirst';
    spi_bus.Mode = 0;
    
    %Configure resistor latter network
    tcon_register = bin2dec('0100 0001 1111 1111'); %[255];
    pot_value = bin2dec('11111111 11111111');  % hex2dec('0F');
    data_write = uint16(bitand(tcon_register,pot_value)); %
    out = writeRead(spi_bus,data_write,'uint16');
    
    %Disables current output, sets value to 1.21 mA
    set_pot_val = uint8(bin2dec('0000 0000'));
    set_pot_val2 = uint8(bin2dec('0001 0000'));
    out = writeRead(spi_bus,[set_pot_val set_pot_val2],'uint8');
    
    %configure wiper increment/decrement commands
    increase_current = uint8(bin2dec('0000 0100'));
    decrease_current = uint8(bin2dec('0000 1000'));
    
    drawnow;
    
else
    disp('Stim Board Not Found')
    return
    
end
board_time = toc(board_time);

%% Get video input
disp('Getting Camera')
camera_time = tic;
if ~exist('vid','var')
    
    vid = videoinput('pointgrey', 1, 'F7_BayerRG8_752x480_Mode0');
end
%configure camera parameters
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

flushdata(vid);
vid.FramesPerTrigger = inf;
start(vid);


flushdata(vid);
frame1 = getdata(vid,1);
frame = imresize(frame1,0.5);
camera_time = toc(camera_time);

%% Boundary
boundary_value = 375;
right_thresh = boundary_value + 100;
left_thresh = boundary_value - 100;

if SMALL_IMAGE
    boundary_value = boundary_value./2;
    right_thresh = right_thresh./2;
    left_thresh = left_thresh./2;
end

fps = 15;
max_time = 2*56*60;
trajectory = zeros(fps*max_time,2);
timestamps = zeros(fps*max_time,5);

%if videos need to be saved for later, uncomment these
store_vid = VideoWriter(['operant_tracker_protocol_' animal_name '.mp4'],'MPEG-4');
open(store_vid);


%% Configure point tracker algo
point_track_time = tic;
pointTracker = vision.PointTracker('NumPyramidLevels',5,'BlockSize',[71 71],'MaxBidirectionalError',5);

old_pos = [];
track_pos = [];
new_coordes = [];
crossings = 0;

[Y,X] = find_gp_colors(frame,color_model,color_thresh);


initialize(pointTracker,median([Y X],1),frame);
old_pos = median([Y X],1);
track_pos = old_pos;

if track_pos(1) <= boundary_value
    boundary_val = right_thresh;
else
    boundary_val = left_thresh;
end

marked = insertMarker(frame,track_pos,'*','color','green');
image(marked);
drawnow;
point_track_time = toc(point_track_time);

%% construct sound classes
%distinct sound levels
sound_time = tic;
sound_levels = [30 40 50 60]+20;
num_levels = numel(unique(sound_levels));

freqs = [6000 8000 10000];
freq_type = zeros(size(freqs));

bands = [4000 6000 8000 10000];
band_type = ones(size(bands));

%load in noise bands
noise_stim = cell(length(bands),1);
Fs = 96000;
GEN_SOUND = false;

band_players = cell(length(bands),length(sound_levels));
tone_players = cell(length(freqs),length(sound_levels));

for idx = 1:length(bands)
    noise_data = load(['noise_band_' num2str(bands(idx)) '.mat']);
    data = noise_data.noise_kept;
    noise_stim{idx} = data./max(abs(data));
    num_pts = noise_data.num_pts;
    
    for idx_lvl = 1:length(sound_levels) %type 0 == tone, type 1 == band
        atten_val = set_calib_data(bands(idx),1,sound_levels(idx_lvl));
        atten_val = atten_val-10;
        player = audioplayer(repmat(data',120,2).*10^(atten_val./20),Fs);
        band_players{idx,idx_lvl} = player;
    end
end

time = 0:1/Fs:(60-1/Fs);
for idx = 1:length(freqs)
    data = sin(2.*pi.*time.*freqs(idx));
    data = repmat(data,2,2);
    
    for idx_lvl = 1:length(sound_levels) %type 0 == tone, type 1 == band
        atten_val = set_calib_data(freqs(idx),0,sound_levels(idx_lvl));
        atten_val = atten_val-10;
        player = audioplayer(data.*10^(atten_val./20),Fs);
        tone_players{idx,idx_lvl} = player;
    end
end

sound_max = 60;

%distinct sounds, freq_type == 0 is tone, band_type == 1 is noise
sounds = [[freqs bands]; [freq_type band_type]];
num_sounds = size(sounds,2);

%number of repetitions per sound
num_sound_reps = 2;

%generate permutations of sounds, levels and repetitions
sounds_rep = repmat(sounds,1,numel(unique(sound_levels)));
sound_levels_rep = reshape(repmat(sound_levels,num_sounds,1)...
    ,num_sounds*num_levels...
    ,1);

protocol = [sounds_rep' sound_levels_rep];
protocol = repmat(protocol,num_sound_reps,1);

num_trial = size(protocol,1);

protocol = [protocol zeros(num_trial,1)];
for idx = 1:num_trial
    protocol(idx,4) = set_calib_data(protocol(idx,1),protocol(idx,2),protocol(idx,3));
end
protocol_times = zeros(num_trial,3);

%randomize protocol info
%standardize seed for reproducibility, or set = 'shuffle' for random
seed = 12281990;%'shuffle'; %
rng(seed);

min_holding = 10;
max_holding = 30;
wait_periods = min_holding+randi((max_holding-min_holding),num_trial,1);
ordering = randsample(1:num_trial,num_trial);

protocol = protocol(ordering,:);

if ENABLE_SILENCE
    disp('Silence Trials enabled')
    num_silence = 10;
    silence_dur = 120;
    silence_trials = randsample(1:num_trial,num_silence);
    
    wait_periods(silence_trials) = silence_dur;
    
    protocol(silence_trials,1) = 0;
    protocol(silence_trials,2) = 2;
    protocol(silence_trials,3) = 0;
    protocol(silence_trials,4) = 120;
end

%animal has this long to cross over before shocking begins
stim_dur_threshold = 30;

state = 'holding'; %1 = holding, 2 = sound, 3 = sound and shock, 4 = freeze

protocol_idx = 1;
time_idx = 1;
start_time = tic;
protocol_start = toc(start_time);
trial_time = toc(start_time);
returning_val = 0;
SOUND_ENABLE = false;
SHOCK_ENABLE = false;
ERROR_ENABLE = false;
sound_time = toc(sound_time);

%% calibration data

% send2gmail('davidmartel07@gmail.com',['Started operant: ' animal_name])
% send2gmail('lampenje@gmail.com',['Started operant: ' animal_name])

try
    total_time = tic;
    rotate_angle_error = 0;
    
    frame = 0;
    
    gp_freeze = 0;
    fail_counter = 0;
    while protocol_idx <= num_trial
        
        %debugging
        %     if protocol_idx == 2
        %         state='freeze'
        %     end
        
        %% Get new frame, align to background image prior to image subtraction
        flushdata([vid]);
        
        frame1 = getdata(vid,1);
        frame = imresize(frame1,0.5);
        
        %% Find learned colors
        [Y,X] = find_gp_colors(frame,color_model,color_thresh);
        
        if isempty(X) || isempty(Y)
            track_pos = old_pos;
        else
            old_pos = track_pos;
            track_pos = median([Y X],1);
        end
        %update point tracker
        [new_coordes,valid] = step(pointTracker,frame);
        if ~valid || sqrt(sum((old_pos-track_pos).^2)) >= 10
            new_coordes = track_pos;
            setPoints(pointTracker,new_coordes);
        end
        
        %save old position, update new position
        track_pos = mean([new_coordes; track_pos]);
        
        
        %% Stor vars, run state machine
        %indicate position of animal, store location and frame time
        marked = insertMarker(frame,[Y X],'*','color','green'); %track_pos
        marked = insertMarker(marked,track_pos,'*','color','red'); %track_pos
        trajectory(time_idx,:) = track_pos;
        frame_time = toc(start_time);
        
        %Update timestamps matrix
        timestamps(time_idx,1) = frame_time;
        timestamps(time_idx,2) = crossings;
        
        if strcmp(state,'holding')
            timestamps(time_idx,3) = 0;
        elseif strcmp(state,'sound')
            timestamps(time_idx,3) = 1;
        elseif strcmp(state,'silence')
            timestamps(time_idx,3) = 3;
        elseif strcmp(state,'freeze')
            timestamps(time_idx,3) = 4;
        else
            timestamps(time_idx,3) = 2;
        end
        timestamps(time_idx,4) = toc(total_time);
        
        trial_time = frame_time - protocol_start;
        
        image_params = {'FontSize',18,'BoxColor','blue','BoxOpacity',0.4,'textcolor','white'};
        marked = insertText(marked,[1 40],['Trial: ' num2str(protocol_idx) '/' num2str(num_trial)],image_params{:});
        
        if ~strcmp(state,'silence')
            marked = insertText(marked,[1 80],['Sound time: ' num2str(wait_periods(protocol_idx))],image_params{:});
            marked = insertText(marked,[1 120],...
                ['Shock time: ' num2str(wait_periods(protocol_idx)+stim_dur_threshold)],image_params{:});
        else
            marked = insertText(marked,[1 80],['Silence ends: ' num2str(wait_periods(protocol_idx))],image_params{:});
        end
        
        marked = insertText(marked,[1 160],['Trial time: ' num2str(trial_time)],image_params{:});
        marked = insertText(marked,[1 200],['State: ' state],image_params{:});
        
        marked = insertText(marked,[230 1],...
            ['Prot. Cross: ' num2str(protocol_times(protocol_idx,2)) ' crossings'],image_params{:});
        marked = insertText(marked,[230 40],...
            ['Curr. Cross: ' num2str(timestamps(time_idx,2)) ' crossings'],image_params{:});
        
        %% State machine
        
        %reset freeze counter
        if gp_freeze < 0
            gp_freeze = 0;
        end
        
        if strcmp(state,'holding') %currently holding period
            
            if trial_time >= wait_periods(protocol_idx)
                state = 'sound';
                protocol_times(protocol_idx,1) = time_idx;
                protocol_times(protocol_idx,2) = crossings;
                
                %enable sound
                SOUND_ENABLE = true;
                SHOCK_ENABLE = false;
                
                if exist('sound_player','var')
                    stop(sound_player);
                end
                
                lvl_idx = find(protocol(protocol_idx,3)==sound_levels);
                %Present tones, or noise band
                if protocol(protocol_idx,2) == 0
                    stim_idx = find(freqs == protocol(protocol_idx,1));
                    sound_player = tone_players{stim_idx,lvl_idx};
                elseif protocol(protocol_idx,2) == 1
                    stim_idx = find(bands == protocol(protocol_idx,1));
                    sound_player = band_players{stim_idx,lvl_idx};
                end
                play(sound_player);
                
            elseif protocol(protocol_idx,2) == 2
                
                state = 'silence';
                stop(sound_player);
                disp('Silence');
                
                SOUND_ENABLE = false;
                SHOCK_ENABLE = false;
                
            end
            
            
        elseif strcmp(state,'silence')
            
            if trial_time > wait_periods(protocol_idx)
                
                protocol_times(protocol_idx,3) = nan;
                protocol_idx = protocol_idx + 1;
                start_time = tic;
                protocol_start = toc(start_time);
                
                state = 'holding';
            end
            
            %% Sound is playing
        elseif strcmp(state,'sound')
            
            %if crossings increase after sound starting, disable stim
            if timestamps(time_idx,2) > protocol_times(protocol_idx,2)
                
                writeDigitalPin(device,current_pin,false)
                stop(sound_player);
                
                disp('Success!');
                SOUND_ENABLE = false;
                SHOCK_ENABLE = false;
                
                start_time = tic;
                protocol_start = toc(start_time);
                
                state = 'holding';
                protocol_times(protocol_idx,3) = 1;
                protocol_idx = protocol_idx + 1;
                
            elseif trial_time > wait_periods(protocol_idx) + stim_dur_threshold

                writeDigitalPin(device,current_pin,true)
                SHOCK_ENABLE = true;
                
                disp('Fail!');
                state = 'shock';
                fail_counter = fail_counter + 1;
                protocol_times(protocol_idx,3) = 0;
                
                if fail_counter >= 30
                    state = 'freeze';
                end
                
            end
            
            %% sound is playing and shock enabled
        elseif strcmp(state,'shock')
            if timestamps(time_idx,2) > protocol_times(protocol_idx,2) || ...
                    trial_time > wait_periods(protocol_idx) + sound_max
                
                %detect freezing behavior
                if trial_time > wait_periods(protocol_idx) + sound_max
                    gp_freeze = gp_freeze + 1;
                else
                    gp_freeze = gp_freeze - 2;
                end
                
                if gp_freeze >= 10
                    state = 'freeze';
                else
                    state = 'holding';
                end
                
                writeDigitalPin(device,current_pin,false)
                
                stop(sound_player);
                SOUND_ENABLE = false;
                SHOCK_ENABLE = false;
                
                protocol_idx = protocol_idx + 1;
                start_time = tic;
                protocol_start = toc(start_time);
                
                
            end
            
        elseif strcmp(state,'error')
            
            %disable camera, exit looping
            
            protocol_idx = num_trial + 1;
            
            writeDigitalPin(device,current_pin,false)
            
            stop(sound_player);
            SOUND_ENABLE = false;
            SHOCK_ENABLE = false;
            ERROR_ENABLE = true;
            
        elseif strcmp(state,'freeze')
            
            protocol_idx = num_trial + 1;
            
            writeDigitalPin(device,current_pin,false)
            
            stop(sound_player);
            SOUND_ENABLE = false;
            SHOCK_ENABLE = false;
            ERROR_ENABLE = true;
            
        end
        
        %% markers and such on frame
        if SOUND_ENABLE
            marked = insertText(marked,[250 80],['SOUND'],'FontSize',18,'BoxColor','green','BoxOpacity',0.4,'textcolor','blue');
        end
        
        if SHOCK_ENABLE
            marked = insertText(marked,[250 120],['SHOCK'],'FontSize',18,'BoxColor','yellow','BoxOpacity',0.4,'textcolor','red');
        end
        
        if ERROR_ENABLE
            
            if strcmp(state,'freeze')
                marked = insertText(marked,[250 160],['FREEZING'],'FontSize',18,'BoxColor','red','BoxOpacity',0.4,'textcolor','white');
                
            else
                marked = insertText(marked,[250 160],['ERROR'],'FontSize',18,'BoxColor','red','BoxOpacity',0.4,'textcolor','white');
            end
            
        end
        
        %% update boundary crossings
        if track_pos(1) > right_thresh && old_pos(1) <= right_thresh && boundary_val == right_thresh
            crossings = crossings + 1;
            boundary_val = left_thresh;
        elseif track_pos(1) < left_thresh && old_pos(1) >= left_thresh && boundary_val == left_thresh
            crossings = crossings + 1;
            boundary_val = right_thresh;
        end
        
        if SMALL_IMAGE
            red_line = [boundary_val 0 boundary_val 480];
        else
            red_line = [boundary_val 0 boundary_val 240];
        end
        
        marked = insertShape(marked,'line',...
            red_line,...
            'linewidth',3,'color','red');
        marked = insertText(marked,[1 1],['Crossings: ' num2str(crossings)],'FontSize',18,'BoxColor','blue','BoxOpacity',0.4,'textcolor','white');
        
        
        %% show frame, save video
        image(marked);
        drawnow;
        writeVideo(store_vid,frame);
        flushdata([vid]); 
        
        time_idx = time_idx + 1;
        frame = 0;
    end
    
    %% Deactivate hardware
    stop(vid);
    flushdata(vid);
    delete vid
    clear vid
    
    release(pointTracker);   
    close(store_vid);    
    stop(sound_player);
    
    
    %% save collected data
    save_vars = {'animal_name','date_str','cur_time','timestamps','trajectory',...
        'protocol','protocol_times','wait_periods','ordering'};
    
    animal_info.animal = animal_name;
    animal_info.date = date_str;
    animal_info.time = cur_time;
    animal_info.timestamps = timestamps;
    animal_info.trajectory = trajectory;
    animal_info.protocol = protocol;
    animal_info.protocol_times = protocol_times;
    animal_info.wait_periods = wait_periods;
    
    save(save_file,'animal_info');
    save([animal_folder filesep date_str filesep 'day_info.mat'],save_vars{:});
    
    if strcmp(state,'holding')
        state = 'success';
    end
    sendmail(email_address,['Operant Testing Finished: ' animal_name ', state: ' state],'Please make sure to check on guinea pig') ;
    sendmail('the-shore-lab@umich.edu',['Operant Testing Finished: ' animal_name ', state: ' state],'Please make sure to check on guinea pig') ;
    
catch
    
    disp('Error occured')
    sendmail('the-shore-lab@umich.edu',['Error Occured on: ' animal_name],'Please make sure to check on guinea pig') ;
    
end


