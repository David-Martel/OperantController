clear
clc
close all

data = load('noise_spec.mat','SHRL0522');
noise_spec = data.SHRL0522;
nfreq = noise_spec.freq;
ndb = noise_spec.dbspl;

data = load('tone_spec.mat','tone_data');
tone_spec = data.tone_data;
tfreq = tone_spec.freq;
tdb = tone_spec.dBSPL1;
atten = tdb+tone_spec.flat_atten1;

figure(1)
clf(1)
hold on
plot(nfreq,ndb,'b')
plot(tfreq,atten,'g')
plot(tfreq,tdb,'r')


