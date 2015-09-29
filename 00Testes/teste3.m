clear all, close all, clc


syms k1 k2 FyMax

z = k1*(k1/(3*k2))^(0.5) - k2*(k1/(3*k2))^(3/2) - FyMax;

k2Sol = solve(z==0,k2) % Solução do k2