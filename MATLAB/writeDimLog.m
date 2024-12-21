clear
clc

% v = round(logspace(0,2.602,801))

t1 = round(0:1:800)
% t = [t1 round(linspace(26, 100, length(v) - length(t1)))]
writematrix(t1, "dimLog.csv");