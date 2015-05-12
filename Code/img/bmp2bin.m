clear all, close all, clc;

fh = fopen('cameraman.bin', 'w');
fwrite(fh, imread('cameraman.bmp'));
fclose(fh);