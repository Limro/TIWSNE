clear all, close all, clc;

fh = fopen('cameraman.bin', 'r');
image = vec2mat(fread(fh), 256)' / 256;
imshow(image);
fclose(fh);