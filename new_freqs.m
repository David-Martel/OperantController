
clear
clc

close all

aud_file_dir = fullfile(pwd,'aud_files');
if ~exist(aud_file_dir,'dir')
    mkdir(aud_file_dir)
end

% Fs = 100000;
% freq_vector = 0:1:(Fs-1);

freq_vector = 4000:30000;

fmin = 6000;
fmax = 26000;

num_freq = 10;

fscale = (fmax-fmin)/num_freq;

freq_spaces = linspace(log2(fmin),log2(fmax),num_freq);
low_freqs = 2.^(freq_spaces);
high_freqs = (1+1/num_freq).*low_freqs;

freq_order = sort([low_freqs high_freqs],'ascend');

new_sig = zeros(size(freq_vector));
iter = false;
sort_iter = 1;
idx = 1;
while idx <= length(freq_vector)
    
    if freq_vector(idx) >= freq_order(sort_iter)
        sort_iter = sort_iter + 1;
        iter = ~iter;
    end
    if sort_iter == length(freq_order)
        idx = length(freq_vector) + 1;
    else
        
        new_sig(idx) = iter;
        
        idx = idx + 1;
    end
end

%make noise bands
aud_file_type = 'wav';
aud_file_rate = 96000;

duration = 30;

Fs = 100000;
freqs_array = 0:1:(Fs/2-1);
freq_list = round([low_freqs' high_freqs']);
sound_store = cell(num_freq,1);
aud_play_store = cell(num_freq,1);
for idx = 1:num_freq
    
    noise = randn(Fs,1);
    noise = noise./max(abs(noise));
    
    noise_fft = fft(noise,Fs);
    noise_fft = noise_fft(1:end/2);
    
    noise_fft(freqs_array <= freq_list(idx,1)) = 0;
    noise_fft(freqs_array > freq_list(idx,2)) = 0;
    
    noise_fft = [noise_fft; flipud(noise_fft(2:end))];
    %noise_fft = movmean(noise_fft,3,'endpoint','discard');
    
    noise_band = real(ifft(noise_fft,Fs));
    noise_band = noise_band./max(abs(noise_band));
    
    sound_store{idx} = noise_band;
    
    play_band = resample(noise_band,aud_file_rate,Fs);
    play_band = play_band./max(abs(play_band));
    play_band = repmat(play_band,duration,1);
    
    audiowrite(fullfile(aud_file_dir,sprintf('sound_%d.%s',freq_list(idx,1),aud_file_type)),play_band,aud_file_rate);
    
    aud_play_store{idx} = audioplayer(play_band,aud_file_rate);
    
end

% plot things
figure(1)
clf(1)
hold on
plot(freq_vector,new_sig)
xlim([fmin/2 fmax*2])
set(gca,'xscale','log')



%%
base_dir = pwd;
% system(['"' base_dir '\nircmd.exe" setsysvolume 65535 &']) %max volume = 16 bit
% drawnow;
% 
% play(aud_play_store{1})


noise = randn(Fs,1);

noise_fft = fft(noise,Fs);
noise_fft = noise_fft(1:end/2);
noise_fft(freqs_array>= 30000) = 0;
noise_fft = [noise_fft; flipud(noise_fft(2:end))];
noise = real(ifft(noise,Fs));

noise = noise./max(abs(noise));
play_band = repmat(noise,duration,1);
play_band = resample(play_band,aud_file_rate,Fs);
audiowrite(fullfile(aud_file_dir,sprintf('sound_%s.%s','BBN'...
    ,aud_file_type)),play_band,aud_file_rate);

bbn_player = audioplayer(play_band,aud_file_rate);

system(['"' base_dir '\nircmd.exe" setsysvolume 65535 &']) %max volume = 16 bit
drawnow;

play(bbn_player)
pause(0.01);

% rlist = audiodevinfo;
% rlist = rlist.input;
% rlist.Name

%% build sound recorder spectrum analyzer
dev = audiorecorder(aud_file_rate,16,1,1);
record(dev);
start_time = tic;
derp = true;
while derp
    cur_time = toc(start_time);
    if cur_time > 10
        derp = false;
    end
end
stop(dev);
sound_data = getaudiodata(dev,'double');

num_sound_pts = length(sound_data);
num_fft_pts = 60000;

num_ave = floor(num_sound_pts/num_fft_pts);

sound_data_short = (sound_data(1:num_fft_pts*num_ave))';

sound_data_short = reshape(sound_data_short,num_fft_pts,num_ave);
sound_data_short = sound_data_short';

sound_spec = real(fft(sound_data_short,num_fft_pts,2));
sound_spec = sound_spec(:,1:num_fft_pts/2);
figure(2)
clf(2)
plot(nanmean(sound_spec))





