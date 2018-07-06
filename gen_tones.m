clear
clc

close all

Fs = 96000;
tone_list = 4000:1000:25000;

duration = 15;

time_vec = 0:1/Fs:(30-1/Fs);

ramp_time = 100/1000; %10ms
ramp_per = (ramp_time/duration);
%window_func = tukeywin(Fs*duration,ramp_per);
%window_func = hamming(2.*Fs*duration);
noise = randn(size(time_vec));
noise = 0.01.*noise;

base_dir = pwd;
atten = floor(2^16.*10^(-0/20));
system(['"' base_dir '\nircmd.exe" setsysvolume ' num2str(atten) ' &']) %max volume = 16 bit
drawnow;

tone_store = cell(length(tone_list),1);
for idx = 1:length(tone_list)
    
    tone_sig = sin(2.*pi.*tone_list(idx).*time_vec);
%     tone_sig = tone_sig + noise;
%     tone_sig = tone_sig./max(abs(tone_sig));
%     
    if tone_list(idx) < 10000
    atten_fac = 10^(-20/20);
    else
        atten_fac = 1;
    end
    
    tone_store{idx} = audioplayer(atten_fac.*tone_sig,Fs);
    
end



