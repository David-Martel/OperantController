

clear
clc
close all


Fs = 96000;

time = 0:1/Fs:(10-1/Fs);

freq = 1000;

sound_sig = sin(2.*pi.*freq.*time);
sig_player = audioplayer(sound_sig,Fs);

play(sig_player);









