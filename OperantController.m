function varargout = OperantController(varargin)
% OPERANTCONTROLLER MATLAB code for OperantController.fig
%      OPERANTCONTROLLER, by itself, creates a new OPERANTCONTROLLER or raises the existing
%      singleton*.
%
%      H = OPERANTCONTROLLER returns the handle to a new OPERANTCONTROLLER or the handle to
%      the existing singleton*.
%
%      OPERANTCONTROLLER('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in OPERANTCONTROLLER.M with the given input arguments.
%
%      OPERANTCONTROLLER('Property','Value',...) creates a new OPERANTCONTROLLER or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before OperantController_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to OperantController_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help OperantController

% Last Modified by GUIDE v2.5 13-Jun-2018 16:27:31

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @OperantController_OpeningFcn, ...
    'gui_OutputFcn',  @OperantController_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before OperantController is made visible.
function OperantController_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to OperantController (see VARARGIN)



current_date = datestr(now,'mmm-dd-yyyy_HH-MM-SS');
handles.current_date = current_date;

handles.test_day.String = current_date;
drawnow;

%
set(handles.image_feed,'XTickLabel',{});
set(handles.image_feed,'XTick',[]);
set(handles.image_feed,'YTick',[]);
set(handles.image_feed,'YTickLabel',{});
set(handles.image_feed,'Box','on');

offset = 720/2;
start_pos = offset/2;
image_height = 480;

right_pos = [offset+start_pos 0 offset+start_pos image_height];
left_pos = [start_pos 0 start_pos image_height];
handles.line_matrix = [left_pos; right_pos];

handles.image_height = image_height;

set(handles.left_slider,'Value',start_pos);
set(handles.left_slider,'Min',0);
set(handles.left_slider,'Max',offset);

set(handles.right_slider,'Value',offset+start_pos);
set(handles.right_slider,'Min',offset);
set(handles.right_slider,'Max',offset*2);
guidata(hObject,handles);
drawnow;

%
protocol_types = {'Training','Baseline','Tinnitus','Other'};
set(handles.proto_type,'String',protocol_types);
set(handles.proto_type,'Value',1);
drawnow;

% Lab specific, computer specific information
shore_comps = {'khri-jim','khri-305971'};
altshuler_comps = {'khri-314461','khri-314463',...
    'khri-314464','khri-314465','khri-314466'};

%computer specific items
[~,name] = system('hostname');
name(end) = [];
name = lower(name);
handles.computer_name = name;

set(handles.comp_name,'String',sprintf('Computer: %s',name));
drawnow;

config_email();


%needed to send email to users
if ismember(name,shore_comps)
    handles.user_email_addr = 'damartel@umich.edu';
    home_server = '\\maize.umhsnas.med.umich.edu\khri-ses-lab\Applications';
    video_construct = {'pointgrey', 1, 'F7_BayerRG8_752x480_Mode0'};
elseif ismember(name,altshuler_comps)
    handles.user_email_addr = 'shuler@umich.edu';
    home_server = '\\maize.umhsnas.med.umich.edu\khri-alt-lab\Applications';
    video_construct = {'pointgrey', 1, 'F7_BayerRG8_752x480_Mode0'};
    
else
    handles.user_email_addr = 'the.khri.email@gmail.com';
    home_server = '';
end
handles.video_construct = video_construct;
guidata(hObject, handles);
drawnow;

%phone home to see if main data store is present, and do version checking
if ~exist(home_server,'dir')
    uiwait(msgbox('Warning: home server not found.'));
else
    version_check = load(fullfile(home_server,'OC_Version.mat'));
    version_check = version_check.version_check;
end
version_runtime = 0.9;
if version_runtime ~= version_check
    uiwait(msgbox('Warning: Newer code available. Please install.'));
end

% folder for data, getting required files
base_dir = 'C:\OperantSetup\';
if ~exist(base_dir,'dir')
    mkdir(base_dir);
end
handles.base_dir = base_dir;

%calibration file
calib_folder = fullfile(base_dir,'Calibration_Data');
if ~exist(calib_folder,'dir')
    mkdir(calib_folder);
end

calib_data_file = fullfile(calib_folder,'noise_spec.mat');%['calibration_data_' name '.xlsx']
handles.calib_file = calib_data_file;
if ~exist(handles.calib_file,'file')
    uiwait(msgbox('Get calibration data before running program'));
end

%data storage location
data_dir_def = fullfile(base_dir,'Data');
if ~exist(data_dir_def,'dir')
    mkdir(data_dir_def);
end
handles.save_dir_str.String = sprintf('Save Dir: %s',data_dir_def);

handles.save_dir = data_dir_def;
guidata(hObject, handles);

%animal color model
color_dir = 'Color_Data';
color_save = fullfile(base_dir,color_dir);
if ~exist(color_save,'dir')
    mkdir(color_save);
end
handles.color_save = color_save;
guidata(hObject, handles);

%default saving behavior is to save protocol data and not video data
handles.save_proto_data.Value = 1;
handles.save_vid_data.Value = 0;
handles.no_acclim.Value = 1;

%update strigns
handles.shock_str.String = 'SHOCK DISABLED';
handles.shock_str.ForegroundColor = [0 0 1]; %blue
handles.sound_str.String = 'SOUND DISABLED';
handles.shock_str.ForegroundColor = [0 1 1]; %green

%testing parameters
handles.tracking = false;
handles.animal_name = 'Animal 1';
handles.prev_trial_str = [];
handles.protocol_type = 'Training';

%run parameters
handles.left_pos = [];
handles.right_pos = [];

handles.proto_state = 'acclim'; %holding, waiting, freezing, sound, shock, silence
handles.crossings = 0;
handles.boundary = [];
handles.tracking = false;
handles.image = [];

%% Configure hardware
handles.SHOCK_STATE = false;
handles.current_lev = 1.21; %default from GP experiments

[hObject,handles] = config_shocker(hObject, handles);

handles.SOUND_STATE = false;
handles.SILENCE_TRIALS = false;

%Configure camera
[hObject,handles] = config_camera(hObject, handles);
% Update handles structure
guidata(hObject, handles);

%Configure sound
[hObject,handles] = config_sound(hObject,handles);

% handles.vid_timer = timer(...
%     'ExecutionMode','fixedRate',...
%     'Period',1/20,...
%     'StartDelay',1); %,...
% handles.vid_timer.TimerFcn= @(~,~)update_image(hObject,handles);

% Choose default command line output for OperantController
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes OperantController wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = OperantController_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% vid = handles.vid;
% close(vid);
% delete(vid);
% clear vid handles.vid;

% Get default command line output from handles structure
varargout{1} = handles.output;








%% Input box handling functions
function config_email()

shore_lab = 'the.khri.email@gmail.com';
shore_lab_pass = 'rwaawkldjgaauslq';


setpref('Internet','SMTP_Server','smtp.gmail.com');
setpref('Internet','E_mail',shore_lab);

setpref('Internet','SMTP_Username',shore_lab);
setpref('Internet','SMTP_Password',shore_lab_pass);

props = java.lang.System.getProperties;
props.setProperty('mail.smtp.auth','true');
props.setProperty('mail.smtp.socketFactory.class', 'javax.net.ssl.SSLSocketFactory');
props.setProperty('mail.smtp.socketFactory.port','465');
drawnow;


function success = animal_name_test(animal_name)
%test for bad characters in animal name
bad_chars = {':','\','/','#','%','&','{','}','<','>','*',...
    '?',' ','$','!','~','@','_','-','+','=','(',')'};
if any(contains(animal_name,bad_chars))
    uiwait(msgbox(['Animal name cannot contain these characters: '...
        strcat(bad_chars)]));
    success = 0;
else
    success = 1;
end

function [hObject,handles] = make_animal_dir(hObject,handles,animal_name)
save_dir = handles.save_dir;

save_dir = fullfile(save_dir,animal_name);
if ~exist(save_dir,'dir')
    uiwait(msgbox('New animal folder being made'));
    mkdir(save_dir);
end

handles.save_dir  = save_dir;
handles.save_dir_str.String = sprintf('Save data dir: %s',save_dir);

guidata(hObject,handles);
drawnow;

function ani_name_Callback(hObject, eventdata, handles)
% hObject    handle to ani_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ani_name as text
%        str2double(get(hObject,'String')) returns contents of ani_name as a double
ani_name = get(handles.ani_name,'String');
if animal_name_test(ani_name)
    
    [hObject,handles] = make_animal_dir(hObject,handles,ani_name);
    handles.animal_name = ani_name;
    
    [hObject,handles] = get_color_model_Callback(hObject, [], handles);
    %handles.get_color_model.Enable = 'off';
    
else
    set(handles.ani_name,'String','Animal 1')
    set(handles.ani_name,'UserData','Animal 1')
end
% Choose default command line output for GD_Protocol_Simulink
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function ani_name_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ani_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in save_proto_data.
function save_proto_data_Callback(hObject, eventdata, handles)
% hObject    handle to save_proto_data (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

value = get(hObject,'Value'); % returns toggle state of save_proto_data
if value
    handles.save_proto_data.Value = 1;
else
    handles.save_proto_data.Value = 0;
end
guidata(hObject,handles);
drawnow;

% --- Executes on button press in save_vid_data.
function save_vid_data_Callback(hObject, eventdata, handles)
% hObject    handle to save_vid_data (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

value = get(hObject,'Value'); % returns toggle state of save_vid_data
if value
    handles.save_vid_data.Value = 1;
else
    handles.save_vid_data.Value = 0;
end
guidata(hObject,handles);
drawnow;

% --- Executes on selection change in proto_type.
function proto_type_Callback(hObject, eventdata, handles)
% hObject    handle to proto_type (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

contents = cellstr(get(hObject,'String')); % returns proto_type contents as cell array
proto_type =  contents{get(hObject,'Value')}; % returns selected item from proto_type

handles.protocol_type = proto_type;
silence_types = {'Baseline','Tinnitus'};

if ismember(proto_type,silence_types)
    handles.SILENCE_TRIALS = true;
else
    handles.SILENCE_TRIALS = false;
end

guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function proto_type_CreateFcn(hObject, eventdata, handles)
% hObject    handle to proto_type (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function test_day_Callback(hObject, eventdata, handles)
% hObject    handle to test_day (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of test_day as text
%        str2double(get(hObject,'String')) returns contents of test_day as a double

% --- Executes during object creation, after setting all properties.
function test_day_CreateFcn(hObject, eventdata, handles)
% hObject    handle to test_day (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function success = valid_email(email)

success = 0;
email_attr = {'@','.'};

if ~all(contains(email,email_attr))
    uiwait(msgbox('Email address must contain: @ and .'));
    return
end
success = 1;

function user_email_Callback(hObject, eventdata, handles)
% hObject    handle to user_email (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of user_email as text
%        str2double(get(hObject,'String')) returns contents of user_email as a double
% Hints: get(hObject,'String') returns contents of ani_name as text
%        str2double(get(hObject,'String')) returns contents of ani_name as a double
user_email = get(handles.user_email,'String');
if valid_email(user_email)
    
    gsuite = {'gmail.com','umich.edu'};
    if any(contains(user_email,gsuite)) && ~contains(user_email,'+')
        user_email = strrep(user_email,'@','+operant@');
    end
        
    
    handles.user_email_addr = user_email;
    handles.user_email.String = user_email;
else
    set(handles.user_email,'String','PI Email')
    set(handles.user_email,'UserData','PI Email')
end
% Choose default command line output for GD_Protocol_Simulink
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function user_email_CreateFcn(hObject, eventdata, handles)
% hObject    handle to user_email (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in update_save_dir.
function update_save_dir_Callback(hObject, eventdata, handles)
% hObject    handle to update_save_dir (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
save_dir = handles.save_dir;

new_save_dir = uigetdir(save_dir);

handles.save_dir = new_save_dir;
handles.save_dir_str.String = sprintf('Save data dir: %s',new_save_dir);
guidata(hObject,handles);
drawnow;
















%% Tracking and camera functions

function [varargout] = find_animal_local(im,colors,cl_thresh)
%function uses color model and green location in color model to classify
%points as a shade of ear marker green or not. Pts returns empty if less
%than 2% of pixels are classified as green.

BOUNDS = false;

x=size(im,1);
y=size(im,2);
scale = 0.5;
%bad_pts_thresh = 0; %0.1;

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
    varargout{1} = gr_pts;
    
        [Y,X] = find(gr_pts);
        varargout{2} = Y;
        varargout{3} = X;

else
    gr_pts = [];
end


function run_protocol(hObject,handles)
%insert bounds in image
line_matrix = handles.line_matrix;
left_thresh = line_matrix(1,1);
right_thresh = line_matrix(2,1);
boundary_value = mean([left_thresh right_thresh]);

animal_name = handles.animal_name;
email_address =  handles.user_email_addr;
base_dir = handles.base_dir;

    run_time = datestr(now,'mmm-dd-yyyy_HH-MM-SS');
    
%% configure trial parameters
seed = 12281990;%'shuffle'; %
rng(seed);

%need listing of sound types here
aud_play_store = handles.aud_play_store;
freq_spaces = handles.freq_spaces;
if isrow(freq_spaces)
    freq_spaces = freq_spaces';
end
sound_lvls = handles.sound_lvls;
if isrow(sound_lvls)
    sound_lvls = sound_lvls';
end

rep_counts = 3;

num_trial = numel(sound_lvls)*numel(freq_spaces);
test_trials = floor(num_trial/2);
max_trial_reps = rep_counts*numel(sound_lvls)*numel(freq_spaces);

freq_rep = repmat(freq_spaces,numel(sound_lvls),1);
inten_rep = repmat(sound_lvls,numel(freq_spaces),1);
protocol = [freq_rep inten_rep];
protocol = repmat(protocol,rep_counts,1);

ordering = randsample(1:max_trial_reps,test_trials);

protocol = protocol(ordering,:);

%timings, these should be stored in handles
min_holding = 10;
max_holding = 30;
wait_periods = min_holding+randi((max_holding-min_holding),test_trials,1);
sound_present_time = 30;
max_shock = 15;

trial_max = max_shock+sound_present_time+max_holding;
acclim_time = 10*60; %10 minutes at 60 seconds per min

%freezing and stoopid animals
freeze_fail = 25;
dumb_fail = 30;

if handles.SILENCE_TRIALS
    disp('Silence Trials enabled')
    num_silence = 10;
    silence_dur = 120;
    silence_trials = randsample(1:test_trials,num_silence);
    
    wait_periods(silence_trials) = silence_dur;
    
    protocol(silence_trials,1) = 0;
    protocol(silence_trials,2) = 2;
    protocol(silence_trials,3) = 0;
    protocol(silence_trials,4) = silence_dur;
end

%get camera, shocker and sound
vid = handles.vid;
device = handles.device; spi_bus = handles.spi_bus;
current_pin = handles.current_pin;

%% Get Image, find GP
if ~isfield(handles,'color_model')
    [hObject,handles] = get_color_model_Callback(hObject, [], handles);
    handles.get_color_model.Enable = 'off';
end

color_model=handles.color_model;
cl_thresh=handles.cl_thresh;
pointTracker = vision.PointTracker('NumPyramidLevels',5,'BlockSize',[71 71],'MaxBidirectionalError',5);
    
if ~isrunning(handles.vid)
    start(vid);
end

flushdata(vid);
frame = getdata(vid,1);

[pts,X,Y] = find_animal_local(frame,color_model,cl_thresh);

initialize(pointTracker,median([Y X],1),frame);
old_pos = median([Y X],1);
track_pos = old_pos;

if track_pos(1) <= boundary_value
    boundary_val = right_thresh;
    plot_line = line_matrix(2,:);
else
    boundary_val = left_thresh;
    plot_line = line_matrix(1,:);
end
%insert bounds
frame = insertMarker(frame,track_pos,'Color','green','marker','*');
frame = insertShape(frame,'line',plot_line,'Color','Red','LineWidth',2);

%% Update gui labels
%state, trial time, trial number, sound onset time, shock onset time

trial_state_string = {};
set(handles.trial_print,'String',trial_state_string);
drawnow;


%% plot image into axis
set(handles.image_feed,'Units','pixels');
resizePos = get(handles.image_feed,'Position');
frame = imresize(frame,[resizePos(4) resizePos(3)]);
%axes(handles.image_feed);
imshow(frame,'Parent',handles.image_feed);
set(handles.image_feed,'Units','normalized');
guidata(hObject,handles);
drawnow;

state_list = {'acclim','holding','silence','sound','shock','sound-and-shock','freeze'};

if handles.no_acclim.Value
    state = 'acclim'; %'holding'; %1 = holding, 2 = sound, 3 = sound and shock, 4 = freeze
    protocol_idx = 0;
else
    state = 'holding';
    protocol_idx = 1;
end

protocol_times = nan(test_trials,3);

time_idx = 1;


SOUND_ENABLE = false;
SHOCK_ENABLE = false;
ERROR_ENABLE = false;

fps = 15;
max_time = ceil((acclim_time+trial_max*test_trials)*fps*1.1);
trajectory = nan(fps*max_time,2);
timestamps = nan(fps*max_time,5);
crossings = 0;

%ensure sound is enabled
system(['"' base_dir '\nircmd.exe" setsysvolume 65535 &']) %max volume = 16 bit

start_time = tic;
protocol_start = toc(start_time);
trial_time = toc(start_time);

max_exp_time = 60*60; %stop all testing after 1 hour

try
    total_time = tic;
    
    frame = 0;
    
    gp_freeze = 0;
    fail_counter = 0;
    while protocol_idx <= test_trials
        
        
        %% Get new frame, align to background image prior to image subtraction
        flushdata(vid);
        while ~vid.FramesAvailable
            drawnow;
        end
        frame = getdata(vid,1);
        
        %% Find learned colors
        [pts,X,Y] = find_animal_local(frame,color_model,cl_thresh);
        
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
        trajectory(time_idx,:) = track_pos;
        frame_time = toc(start_time);
        
        if frame_time >= max_exp_time
            state = 'time_over';
        end
        
        %Update timestamps matrix
        timestamps(time_idx,1) = frame_time;
        timestamps(time_idx,2) = crossings;
        
        state_code = find(contains(state_list,state),1);
        timestamps(time_idx,3) = state_code;
        
        full_elapsed_time = toc(total_time);
        timestamps(time_idx,4) = full_elapsed_time;
        
        trial_time = frame_time - protocol_start;
        
        set(handles.exp_time,'String',sprintf('Time: %4.2f/%4.0f',full_elapsed_time,max_exp_time));
        set(handles.state_label,'String',sprintf('State: %s',state));
        set(handles.trial_number_str,'String',sprintf('Trials: %d/%d',protocol_idx,test_trials));
        set(handles.trial_time,'String',sprintf('Trial time: %3.2f sec',trial_time));
     	set(handles.cross_str,'String',sprintf('Crossings: %d',crossings));
        
        if strcmp(state,'acclim')
            set(handles.sound_time_str,'String',sprintf('Start: %3.2f sec',acclim_time));
        
        set(handles.estim_time_str,'String',sprintf('EStim: %d sec','---'));
                  
        else
          set(handles.sound_time_str,'String',sprintf('Sound: %d sec',wait_periods(protocol_idx)));
        
        set(handles.estim_time_str,'String',sprintf('EStim: %d sec',wait_periods(protocol_idx) + sound_present_time));
                   
        end

        
        %wait_periods(protocol_idx) + sound_present_time + max_shock;
        
        %set(handles.exp_time_str,'String',sprintf('Total Time: %d',
        %update string parameters on image

%         if ~strcmp(state,'silence')
%             marked = insertText(marked,[1 80],...
%                   ['Sound time: ' num2str(wait_periods(protocol_idx))],image_params{:});
%             marked = insertText(marked,[1 120],...
%                 ['Shock time: ' num2str(wait_periods(protocol_idx)+stim_dur_threshold)],image_params{:});
%         else
%             marked = insertText(marked,[1 80],['Silence ends: ' num2str(wait_periods(protocol_idx))],image_params{:});
%         end

        %% State machine
        
        %reset freeze counter
        if gp_freeze < 0
            gp_freeze = 0;
        end
        
        %acclimation period
        if strcmp(state,'acclim') || protocol_idx == 0
            
            if trial_time >= acclim_time
                state = 'holding';
                protocol_idx = protocol_idx + 1;
            end
            
        elseif strcmp(state,'holding') %currently holding period
            
            if trial_time >= wait_periods(protocol_idx)
                state = 'sound';
                protocol_times(protocol_idx,1) = time_idx;
                protocol_times(protocol_idx,2) = crossings;
                
                %enable sound
                SOUND_ENABLE = true;
                SHOCK_ENABLE = false;
                drawnow;
                
                if exist('sound_player','var')
                    stop(sound_player);
                end

                sound_band = protocol(protocol_idx,1);
                sound_inten = protocol(protocol_idx,2);
                f_idx = find(freq_spaces==sound_band,1);
                i_idx = find(sound_lvls==sound_inten,1);
                
                sound_player = aud_play_store{f_idx,i_idx};
                play(sound_player);
                drawnow;
                
            elseif protocol(protocol_idx,2) == 2
                
                state = 'silence';
                stop(sound_player);
                
                trial_state_string = [trial_state_string; {'Silence'}];
                set(handles.trial_print,'String',trial_state_string);
                drawnow;
                
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
                
                trial_state_string = [trial_state_string; {sprintf('Trial %d: Success!',protocol_idx)}];
                set(handles.trial_print,'String',trial_state_string);
                drawnow;
                
                SOUND_ENABLE = false;
                SHOCK_ENABLE = false;
                drawnow;
                
                start_time = tic;
                protocol_start = toc(start_time);
                
                state = 'holding';
                protocol_times(protocol_idx,3) = 1;
                protocol_idx = protocol_idx + 1;
                
            elseif trial_time > wait_periods(protocol_idx) + sound_present_time
            %animal hasn't crossed, go to sound with shock
            
                writeDigitalPin(device,current_pin,true)
                SHOCK_ENABLE = true;
                SOUND_ENABLE = true;
                
                trial_state_string = [trial_state_string; {sprintf('Trial %d: Fail!',protocol_idx)}];
                set(handles.trial_print,'String',trial_state_string);
                drawnow;
                
                state = 'sound-and-shock';
                fail_counter = fail_counter + 1;
                protocol_times(protocol_idx,3) = 0;
                
                if fail_counter >= dumb_fail
                    state = 'freeze';
                end
                
            end
            
            %% sound is playing and shock enabled
        elseif strcmp(state,'sound-and-shock')
            max_stim = wait_periods(protocol_idx) + sound_present_time + max_shock;
            if timestamps(time_idx,2) > protocol_times(protocol_idx,2) || ...
                    trial_time > max_stim
                
                %detect freezing behavior
                if trial_time > max_stim
                    gp_freeze = gp_freeze + 1;
                else
                    gp_freeze = gp_freeze - 1;
                end
                
                if gp_freeze >= freeze_fail
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
        elseif strcmp(state,'time_over')
            
            protocol_idx = test_trials + 1;            
            writeDigitalPin(device,current_pin,false)           
            stop(sound_player);
            SOUND_ENABLE = false;
            SHOCK_ENABLE = false;
            ERROR_ENABLE = false;
            
        elseif strcmp(state,'error')
            
            protocol_idx = test_trials + 1;            
            writeDigitalPin(device,current_pin,false)           
            stop(sound_player);
            SOUND_ENABLE = false;
            SHOCK_ENABLE = false;
            ERROR_ENABLE = true;
            
        elseif strcmp(state,'freeze')
            
            protocol_idx = test_trials + 1;           
            writeDigitalPin(device,current_pin,false)           
            stop(sound_player);
            SOUND_ENABLE = false;
            SHOCK_ENABLE = false;
            ERROR_ENABLE = true;
            
        end
        
        %% markers and such on frame    
        if SOUND_ENABLE
            handles.sound_str.String = "SOUND ENABLED";
            handles.sound_str.ForegroundColor = [0 1 0]; %color green 
        else
            handles.sound_str.String = 'SOUND DISABLED';
            handles.sound_str.ForegroundColor = [0 1 1]; %green
        end
        drawnow;
        
        if SHOCK_ENABLE
            handles.shock_str.String = "SHOCK ENABLED";
            handles.shock_str.ForegroundColor = [1 0 0]; %color red                 
        else
            % marked = insertText(marked,[250 120],['SHOCK'],'FontSize',18,'BoxColor','yellow','BoxOpacity',0.4,'textcolor','red');
            handles.shock_str.String = 'SHOCK DISABLED';
            handles.shock_str.ForegroundColor = [0 0 1]; %blue    
        end
        drawnow;
        guidata(hObject,handles);
        
        if ERROR_ENABLE
            
%             if strcmp(state,'freeze')
%                 marked = insertText(marked,[250 160],['FREEZING'],'FontSize',18,'BoxColor','red','BoxOpacity',0.4,'textcolor','white');                
%             else
%                 marked = insertText(marked,[250 160],['ERROR'],'FontSize',18,'BoxColor','red','BoxOpacity',0.4,'textcolor','white');
%             end
            
        end
        drawnow;
        
        %% update boundary crossings
        if ismember(state,{'holding'}) %false positives
            
            %cross from right to left
            if track_pos(1) > right_thresh ...
                    && old_pos(1) <= right_thresh ...
                    && boundary_val == right_thresh
                
                crossings = crossings + 1; %keep separate?
                boundary_val = left_thresh; %remove for false pos
                plot_line = line_matrix(1,:); % remove for false pos
                
                %SHOCK_ENABLE = true;
                
            elseif track_pos(1) < left_thresh ...
                    && old_pos(1) >= left_thresh ...
                    && boundary_val == left_thresh
                
                crossings = crossings + 1; %would remove this if state is holding
                boundary_val = right_thresh;
                plot_line = line_matrix(2,:);
                
                %SHOCK_ENABLE = true
                %else
                %SHOCK_ENABLE = false
            end
            
            
        else
            if track_pos(1) > right_thresh ...
                    && old_pos(1) <= right_thresh ...
                    && boundary_val == right_thresh
                crossings = crossings + 1;
                boundary_val = left_thresh;
                plot_line = line_matrix(1,:);
            elseif track_pos(1) < left_thresh ...
                    && old_pos(1) >= left_thresh ...
                    && boundary_val == left_thresh
                crossings = crossings + 1;
                boundary_val = right_thresh;
                plot_line = line_matrix(2,:);
            end
        end
        
        %% plot image into axis
        %indicate position of animal, store location and frame time
        marked = insertMarker(frame,[Y X],'*','color','green'); %track_pos
        marked = insertMarker(marked,track_pos,'*','color','red'); %track_pos
        marked = insertShape(marked,'line',plot_line,'Color','Red','LineWidth',2);

        set(handles.image_feed,'Units','pixels');
        resizePos = get(handles.image_feed,'Position');
        marked = imresize(marked,[resizePos(4) resizePos(3)]);
        %axes(handles.image_feed);
        imshow(marked,'Parent',handles.image_feed);
        set(handles.image_feed,'Units','normalized');
        guidata(hObject,handles);
        drawnow;

        flushdata(vid); 
        
        time_idx = time_idx + 1;
        frame = 0;
        
        
    end
    
    %% Deactivate hardware
    stop(vid);
    flushdata(vid);
%     delete vid
%     clear vid
    
    release(pointTracker);   
%     close(store_vid);    
    stop(sound_player);
    

    if strcmp(state,'holding')
        state = 'success';
    end    
    
    sendmail(email_address,['Operant Testing Finished: ' animal_name ', state: ' state],'Please make sure to check on guinea pig') ;
    sendmail('the-shore-lab@umich.edu',['Operant Testing Finished: ' animal_name ', state: ' state],'Please make sure to check on guinea pig') ;
    
catch MException
    state = 'error';

    error_dir = fullfile(handles.base_dir,'Error Log');
    if ~exist(error_dir,'dir')
        mkdir(error_dir)
    end
    run_time = datestr(now,'mmm-dd-yyyy_HH-MM-SS');
    error_file = fullfile(error_dir,sprintf('ErrorFiles-%s.mat',run_time));
    save(error_file,'MException')
     
    disp('Error occured')
    
    sendmail(email_address,['Operant Testing Finished: ' animal_name ', state: Error'],'Please make sure to check on guinea pig') ;    
    sendmail('the-shore-lab@umich.edu',['Error Occured on: ' animal_name],'Please make sure to check on guinea pig') ;
    
end

    sendmail('davidmartel07+operant@gmail.com','Derp','Derp');
    
    %% save collected data
    save_dir = handles.save_dir;   
    animal_folder = fullfile(save_dir,run_time);
    if ~exist(animal_folder,'dir')
        mkdir(animal_folder);
    end
    
    
    animal_name = [animal_name '_' run_time '_' state '.mat'];
    save_file = fullfile(animal_folder,animal_name);
    
    save_vars = {'animal_name','run_time','timestamps','trajectory',...
        'protocol','protocol_times','wait_periods','ordering'};
    
    animal_info.animal = animal_name;
    animal_info.date = run_time;
    %animal_info.time = cur_time;
    animal_info.timestamps = timestamps;
    animal_info.trajectory = trajectory;
    animal_info.protocol = protocol;
    animal_info.protocol_times = protocol_times;
    animal_info.wait_periods = wait_periods;
    
    save(save_file,'animal_info');
    save([animal_folder filesep run_time filesep 'day_info.mat'],save_vars{:});
    

    %% disable sitmuli
system(['"' base_dir '\nircmd.exe" setsysvolume 0 &']) %max volume = 16 bit
    
% marked = insertText(marked,[250 120],['SHOCK'],'FontSize',18,'BoxColor','yellow','BoxOpacity',0.4,'textcolor','red');
handles.shock_str.String = 'SHOCK DISABLED';
handles.shock_str.ForegroundColor = [0 0 1]; %blue
handles.sound_str.String = 'SOUND DISABLED';
handles.shock_str.ForegroundColor = [0 1 1]; %green

if isrunning(vid)
    stop(vid);
end
if exist('sound_player','var')
    stop(sound_player);
end
writeDigitalPin(device,current_pin,false)
stop(sound_player);




function [hObject,handles] = config_sound(hObject,handles)

aud_file_dir = fullfile(pwd,'aud_files');
if ~exist(aud_file_dir,'dir')
    mkdir(aud_file_dir)
end

% Fs = 100000;
% freq_vector = 0:1:(Fs-1);

freq_vector = 4000:30000;

fmin = 4000;
fmax = 26000;

num_freq = 10;

sound_lvls = [5:5:40]; %sensation level --> need calibration, animal threshold
num_inten = length(sound_lvls);

freq_spaces = linspace(log2(fmin),log2(fmax),num_freq);
low_freqs = 2.^(freq_spaces);
high_freqs = (1+1/num_freq).*low_freqs;

% freq_order = sort([low_freqs high_freqs],'ascend');
% 
% new_sig = zeros(size(freq_vector));
% iter = false;
% sort_iter = 1;
% idx = 1;
% while idx <= length(freq_vector)
%     
%     if freq_vector(idx) >= freq_order(sort_iter)
%         sort_iter = sort_iter + 1;
%         iter = ~iter;
%     end
%     if sort_iter == length(freq_order)
%         idx = length(freq_vector) + 1;
%     else
%         
%         new_sig(idx) = iter;
%         
%         idx = idx + 1;
%     end
% end

%load calibration data
calib_data = load(fullfile(handles.calib_file));
noise_data = calib_data.noise_data;%eh, rename this
noise_data.dbspl = noise_data.dbspl + 20; 
%David Martel: account for unintentional attenuation during calibration

%make noise bands
aud_file_type = 'wav';
aud_file_rate = 96000;

duration = 60; %worse case scenario, max hold + max stim cross

Fs = 100000;
freqs_array = 0:1:(Fs/2-1);
freq_list = round([low_freqs' high_freqs']);
%sound_store = cell(num_freq,1);
aud_play_store = cell(num_freq,num_inten);
for f_idx = 1:num_freq
    
    noise = randn(Fs,1);
    noise = noise./max(abs(noise));
    
    noise_fft = fft(noise,Fs);
    noise_fft = noise_fft(1:end/2);
    
    noise_fft(freqs_array <= freq_list(f_idx,1)) = 0;
    noise_fft(freqs_array > freq_list(f_idx,2)) = 0;
    
    noise_fft = [noise_fft; flipud(noise_fft(2:end))];
  
    noise_band = real(ifft(noise_fft,Fs));
    noise_band = noise_band./max(abs(noise_band));
    
    %sound_store{f_idx} = noise_band;
    
    play_band = resample(noise_band,aud_file_rate,Fs);
    play_band = play_band./max(abs(play_band));
    play_band = repmat(play_band,duration,1);
    
    audiowrite(fullfile(aud_file_dir,sprintf('sound_%d.%s',freq_list(f_idx,1),aud_file_type)),play_band,aud_file_rate);
    
    calib_atten = interp1(noise_data.freq,noise_data.dbspl,geomean(freq_list(f_idx,:)));
    
    for i_idx = 1:num_inten
        
        scale_val = calib_atten-sound_lvls(i_idx); %damartel to fix
         %10.^(-scale_val/20);
        if scale_val > 0
            scale_val = 0;
        end
        
        scale_val = db2mag(scale_val);
        play_band_write = play_band.*scale_val;
        aud_play_store{f_idx,i_idx} = audioplayer(play_band_write,aud_file_rate);
    end
end

handles.aud_play_store = aud_play_store;
handles.freq_spaces = freq_spaces;
handles.sound_lvls = sound_lvls;

% Update handles structure
guidata(hObject, handles);
drawnow;




function [hObject,handles] = config_camera(hObject, handles)


video_construct = handles.video_construct;
if ~isfield(handles,'vid')
    try
        vid = videoinput(video_construct{:});
    catch
        handles=rmfield(handles,'vid');
        
        % Update handles structure
        guidata(hObject, handles);
        imaqreset;
        uiwait(msgbox('Camera not connected; please connect and restart'));
        return
    end
else
    vid = handles.vid;
    clear vid
    delete vid
    imaqreset;
    vid = videoinput(video_construct{:});
    
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

handles.vid = vid;


% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in get_camera.
function get_camera_Callback(hObject, eventdata, handles)
% hObject    handle to get_camera (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if ~isfield(handles,'vid')
    [hObject,handles] = config_camera(hObject, handles);
else
    
%     if ~isrunning(handles.vid)
%         start(handles.vid);
%     else
%         % stop(handles.vid);
%     end
    
%     if strcmp(handles.vid_timer.Running,'off')
%         start(handles.vid_timer);
%     else
%         % stop(handles.vid_timer);
%     end
    
end


% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in tracker.
function tracker_Callback(hObject, eventdata, handles)
% hObject    handle to tracker (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.tracking
    handles.tracking = false;
else
    handles.tracking = true;
end
guidata(hObject,handles);








%% Protocol Execution
% --- Executes on button press in start_protocol.
function start_protocol_Callback(hObject, eventdata, handles)
% hObject    handle to start_protocol (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if ~isfield(handles,'vid')
    uiwait(msgbox('Enable Camera'));
    return
end


handles.increase_current.Enable = 'off';
handles.decrease_current.Enable = 'off';
handles.toggle_shock.Enable = 'off';

run_protocol(hObject,handles);
%do other things to make sure buttons can't be pushed or changed




% --- Executes on button press in pause_stop_proto.
function pause_stop_proto_Callback(hObject, eventdata, handles)
% hObject    handle to pause_stop_proto (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



%% hardware control
function set_current(hObject,handles,cur_level)

function increase_current(hObject,handles)

writeRead(handles.spi_bus,handles.inc_cur,'uint8');
drawnow;

% --- Executes on button press in increase_current.
function increase_current_Callback(hObject, eventdata, handles)
% hObject    handle to increase_current (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.current_lev = handles.current_lev + 0.06;%empircally determined scalar
if handles.current_lev <= 5 %current max
    increase_current(hObject,handles);
else
    handles.current_lev = 5;
end
guidata(hObject,handles);


function decrease_current(hObject,handles)

writeRead(handles.spi_bus,handles.dec_cur,'uint8');
drawnow;

% --- Executes on button press in decrease_current.
function decrease_current_Callback(hObject, eventdata, handles)
% hObject    handle to decrease_current (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.current_lev = handles.current_lev - 0.06;%empircally determined scalar
if handles.current_lev >= 0
    decrease_current(hObject,handles);
end

guidata(hObject,handles);


% --- Executes on button press in toggle_shock.
function toggle_shock_Callback(hObject, eventdata, handles)
% hObject    handle to toggle_shock (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

shock_state = ~(handles.SHOCK_STATE);

if shock_state
    writeDigitalPin(handles.device,handles.current_pin,1)
    handles.shock_str.String = "SHOCK ENABLED";
    handles.shock_str.ForegroundColor = [1 0 0]; %color red
    
else
    writeDigitalPin(handles.device,handles.current_pin,0) ;
    handles.shock_str.String = 'SHOCK DISABLED';
    handles.shock_str.ForegroundColor =[0 0 1];
end
drawnow;

handles.SHOCK_STATE = shock_state;
guidata(hObject,handles);


function [hObject,handles] = config_shocker(hObject, handles)


%is device part of handles?
if ~isfield(handles,'device')
    try
        device = arduino('com3','uno');
        device_found = true;
    catch MException
        
        if contains(MException.message,'MATLAB connection to Arduino')
            if isfield(handles,'device')
                device = handles.device;
                device_found = true;
            else
                uiwait(msgbox('Restart Matlab; error with shocker'));
                handles.increase_current.Enable = 'off';
                handles.decrease_current.Enable = 'off';
                handles.toggle_shock.Enable = 'off';
                return
            end
            
        else
            
            device = [];
            device_found = false;
        end
    end
    
    
    if device_found
        
        %Disable current delivery pin
        [~, name] = system('hostname');
        name(end) = '';
        
        if ismember(name,{'khri-5434dmdesk','KHRI-305971','DESKTOP-KHTHDBD'})
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
        drawnow;
        
        %Disables current output, sets value to 2.23 mA
        set_pot_val = uint8(bin2dec('0000 0000'));
        set_pot_val2 = uint8(bin2dec('0001 1100'));
        out = writeRead(spi_bus,[set_pot_val set_pot_val2],'uint8');
        drawnow;
        
        %configure wiper increment/decrement commands
        increase_current = uint8(bin2dec('0000 0100'));
        decrease_current = uint8(bin2dec('0000 1000'));
        
        handles.inc_cur = increase_current;
        handles.dec_cur = decrease_current;
        handles.device = device;
        handles.spi_bus = spi_bus;
        
        handles.current_pin = current_pin;
        
        handles.increase_current.Enable = 'on';
        handles.decrease_current.Enable = 'on';
        handles.toggle_shock.Enable = 'on';
        
        
        guidata(hObject,handles);
        
        drawnow;
        
    else
        disp('Stim Board Not Found')
        
        handles.increase_current.Enable = 'off';
        handles.decrease_current.Enable = 'off';
        handles.toggle_shock.Enable = 'off';
        
        return
        
    end
else
    handles.increase_current.Enable = 'on';
    handles.decrease_current.Enable = 'on';
    handles.toggle_shock.Enable = 'on';
end

% --- Executes on button press in get_shock.
function get_shock_Callback(hObject, eventdata, handles)
% hObject    handle to get_shock (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[hObject,handles] = config_shocker(hObject, handles);
drawnow;
guidata(hObject,handles);

if isfield(handles,'device')
    handles.get_shock.Enabled = 'off';
end









%% Update trial progress on GUI
% --- Executes on selection change in trial_print.
function trial_print_Callback(hObject, eventdata, handles)
% hObject    handle to trial_print (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns trial_print contents as cell array
%        contents{get(hObject,'Value')} returns selected item from trial_print


% --- Executes during object creation, after setting all properties.
function trial_print_CreateFcn(hObject, eventdata, handles)
% hObject    handle to trial_print (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function left_slider_Callback(hObject, eventdata, handles)
% hObject    handle to left_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
left_pos = round(get(hObject,'Value'));

line_matrix = handles.line_matrix;
line_matrix(1,:) = [left_pos 0 left_pos handles.image_height];
handles.line_matrix = line_matrix;
guidata(hObject,handles);



% --- Executes during object creation, after setting all properties.
function left_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to left_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function right_slider_Callback(hObject, eventdata, handles)
% hObject    handle to right_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
right_pos = get(hObject,'Value');

line_matrix = handles.line_matrix;
line_matrix(2,:) = [right_pos 0 right_pos handles.image_height];
handles.line_matrix = line_matrix;
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function right_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to right_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in get_color_model.
function [hObject,handles] = get_color_model_Callback(hObject, ~, handles)
% hObject    handle to get_color_model (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

animal_name = handles.animal_name;
if ~contains(animal_name,{'Animal Name','Animal '})
    color_save =handles.color_save;
    color_model_file = fullfile(color_save,sprintf('color_model_%s*.mat',animal_name));
    file_list = dir(color_model_file);
    if ~isempty(file_list)
        
        model = load(fullfile(color_save,file_list(1).name),'color_model','cl_thresh');
        color_model = single(model.color_model);
        cl_thresh = single(model.cl_thresh);
        
    else
        TRAIN_MODEL = true;
        uiwait(msgbox('Train color model before running trial'));
        color_model = [];
        cl_thresh = []; 
    end
    
    handles.color_model = color_model;
    handles.cl_thresh = cl_thresh;
    
else
    uiwait(msgbox('Specify animal before loading color model'));
    
end
guidata(hObject,handles);
drawnow;


% --- Executes on button press in no_acclim.
function no_acclim_Callback(hObject, eventdata, handles)
% hObject    handle to no_acclim (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

value = get(hObject,'Value'); % returns toggle state of save_vid_data
if value
    handles.no_acclim.Value = 1;
else
    handles.no_acclim.Value = 0;
end
guidata(hObject,handles);
drawnow;

% Hint: get(hObject,'Value') returns toggle state of no_acclim



function cur_level_str_Callback(hObject, eventdata, handles)
% hObject    handle to cur_level_str (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of cur_level_str as text
%        str2double(get(hObject,'String')) returns contents of cur_level_str as a double


% --- Executes during object creation, after setting all properties.
function cur_level_str_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cur_level_str (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
