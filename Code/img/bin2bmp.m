clear all, close all, clc;
addpath('C:\Users\Rune\Desktop\shared tinyos')
fh = fopen('reimage.bin', 'r');
image = vec2mat(fread(fh), 256)' / 256;
imshow(image);
fclose(fh); 