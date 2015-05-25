clear all, close all, clc;
addpath('F:\TinyOS\TinyOSShared')
fh = fopen('reimage.bin', 'r');
image = vec2mat(fread(fh), 256)' / 256;
imshow(image);
fclose(fh); 