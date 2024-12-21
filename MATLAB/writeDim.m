clear
clc

v1 = 100:-1:0;
v2 = 0:1:99;
v = [v2 v1];
writematrix(v, "dim2.csv");