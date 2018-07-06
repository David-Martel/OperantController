
clear
clc

close all

aud_file_dir = fullfile(pwd,'aud_files');
if ~exist(aud_file_dir,'dir')
    mkdir(aud_file_dir)
end

% Fs = 100000;
% freq_vector = 0:1:(Fs-1);

freq_vector = 4000:32000;

fmin = 6000;
fmax = 26000;

num_freq = 15;

fscale = (fmax-fmin)/num_freq;

freq_spaces = linspace(log2(fmin),log2(fmax),num_freq);
low_freqs = 2.^(freq_spaces);
high_freqs = (1+1/num_freq).*low_freqs;

%% Plot spectra
freq_order = sort([low_freqs high_freqs],'ascend');

gap_bands = [8000 10000 12000 14000 16000 18000 20000 30000];

old_freqs = [6000 8000 10000];

old_bands = [4000 8000 6000 12000 8000 16000 10000 20000];

new_sig = zeros(size(freq_vector));
gap_sig = zeros(size(freq_vector));
iter = false;
gap_iter = false;
gap_track = 1;
sort_iter = 1;
idx = 1;
while idx <= length(freq_vector)
    
    
    if sort_iter <= length(freq_order)
        if freq_vector(idx) >= freq_order(sort_iter)
            sort_iter = sort_iter + 1;
            iter = ~iter;
        end
    end
    
    if gap_track<= length(gap_bands)
    if (freq_vector(idx) > gap_bands(gap_track))
        gap_track = gap_track + 1;
        gap_iter = ~gap_iter;    
    end
    end
    
        new_sig(idx) = iter;
        gap_sig(idx) = gap_iter;
        idx = idx + 1;   
    
end

noise_freq = [min(freq_vector) 4000 7000 10000 max(freq_vector)];
noise_spec = 1.5.*[0 0 1 0 0];

% plot things
figure(1)
clf(1)
hold on
area(noise_freq,noise_spec,'facecolor',0.8.*ones(3,1),'edgecolor','white','linewidth',1.5)

plot(freq_vector,1.2.*gap_sig,'r','linewidth',1.5)
plot(freq_vector,new_sig,'b')

stem(old_freqs,0.8.*ones(size(old_freqs)),'color',[0 1 0.5],'linewidth',1.5)
%stairs(old_bands,0.8.*ones(size(old_bands)),'color',[0 1 0.5]);

xlim([fmin/2 fmax*2])
set(gca,'xscale','log')

xtick_loc = 4000.*(2.^(0:.33:4));
%xtick_loc(xtick_loc>fmax) = [];

set(gca,'xtick',xtick_loc);
set(gca,'xticklabel',strsplit(num2str(xtick_loc./1000,2),' '))
set(gca,'color','k')
set(gcf,'color','k')
legend('Noise Spec','Gap Bands','Operant Bands','location','northwest')
xlabel('Freq (kHz)')
ylabel('None')
title('Relevant Spectra for tinnitus testing')

%% Make signals
%make noise bands
% %aud_file_type = 'mp4';
% aud_file_rate = 48000;
% 
% duration = 30;
% 
% Fs = 100000;
% freqs_array = 0:1:(Fs/2-1);
% freq_list = round([low_freqs' high_freqs']);
% sound_store = cell(num_freq,1);
% for idx = 1:num_freq
%     
%     noise = randn(Fs,1);
%     noise = noise./max(abs(noise));
%     
%     noise_fft = fft(noise,Fs);
%     noise_fft = noise_fft(1:end/2);
%     
%     noise_fft(freqs_array <= freq_list(idx,1)) = 0;
%     noise_fft(freqs_array > freq_list(idx,2)) = 0;
%     
%     noise_fft = [noise_fft; flipud(noise_fft(2:end))];
%     %noise_fft = movmean(noise_fft,3,'endpoint','discard');
%     
%     noise_band = real(ifft(noise_fft,Fs));
%     
%     sound_store{idx} = noise_band;
%     
%     play_band = resample(noise_band,aud_file_rate,Fs);
% %    audiowrite(fullfile(aud_file_dir,sprintf('sound_%d.%s',freq_list(idx,1),aud_file_type)),repmat(play_band,duration,1),aud_file_rate);
%     
% end






