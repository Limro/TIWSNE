clear all, close all, clc;

fh = fopen('decompressed.bin', 'r');
image = vec2mat(fread(fh), 256)' / 256;
imshow(image);
fclose(fh);