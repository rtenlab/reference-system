%%
clc; clear all; close all;

default_data = importdata('test_5.filterline.yml', ' ');
picas_data = importdata('test_59.filterline.yml', ' ');

figure(1); hold on;
plot(default_data.data, 'b');
plot(picas_data.data, 'r');