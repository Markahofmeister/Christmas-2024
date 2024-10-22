clear
clc

wav = fread(fopen('beethoven-stereo-2_duplicate-channels.wav', 'r'), [1 , Inf], 'uint8');
t = 1 : 1 : length(wav);
x = length(t) / 2;
offset = 45;

% Truncate off header
wavT = wav(offset:end);

% Split into two vectors each with half length, 
wav_2 = reshape(wavT, [], length(wavT) / 2);

% Pull out only left channel samples
wav_3 = wav_2(1,1:2:end);
wav_4 = wav_2(2,1:2:end); 

% Make new matrix with only left channel samples
len = length(wav_3);
wav_5 = zeros(2,len);
wav_5(1,:) = wav_3;
wav_5(2,:) = wav_4;

% Interleave samples into one long vector. 
wav_6 = reshape(wav_5, 1, []);

% Put header back on 
newWav = wav(1:offset - 1);
wav_6 = [newWav wav_6];

% Write to new reduced file. 
file = fopen('beethoven-stereo-2_duplicate-channels-reduced_with-header.wav', 'w');
fwrite(file, wav_6);
fclose(file);

