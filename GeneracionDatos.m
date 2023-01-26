% Generacion de datos de tareas

clc; 
clear all;

h = [   4;
        5;
        8];
C = [   1;
        2;
        2];
  
    
color(1,:) = [0.1, 0.4, 0.8];
color(2,:) = [1.0, 0.1, 0.1];
color(3,:) = [0.6, 0.8, 0.2];

%     cBlue = [18/255 104/255 179/255];
%     cRed = [237/255 36/255 38/255];
%     cGreen = [155 190 61]/255;
    
    
N = length(h);
H = 1;
t_min = 1;

for i = 1:N
    H = lcm(H, h(i));
    t_min = gcd(t_min, C(i));
end

gran = 1e-3;
h = h*gran;
C = C*gran;
H = H*gran;
t_min = t_min*gran;

U = sum(C./h);
U_lub = N*(2^(1/N)-1);

% Funcion de tranferencia de la planta
%                   1
% P(s) = ------------------------
%            a*s^2 + b*s + c
a = [8.878e-12  1.776e-11   2.663e-11]/0.0274;
b = [1.291e-5   2.583e-5    3.874e-5]/0.0274;
c = 0.0007648*[1 1 1]/0.0274;


% Controlador PID
%  u(s) = (P + I/s+ D*s)*e(s) -> (P + I/s+ D*s/(s/N+1)*e(s) s = W
% e(s) = r(s) - y(s)
% r(s) : referencia, donde queremos llegar
% y(s) : es el valor medido en ese instante y(s) = y_m(s) + ruido(s); y(s)
%
% y(s) = 45° +- 5°
% ruido(s) = 5*cos(wt); w = 2*pi*500;
% D(s) = D*s*e(s) = D d/dt(45 + 5*cos(500*(5))) = 45 + 2500*cos(150°)) 

P = [0.028842   0.035017 0.039862];
I = [5.1249     3.1462   1.4256];
D = 0*[1 1 1];


% [B, prior] = sort(h);
%Intergenarcion de llaves para planificador
[Dt idx] = planner('RM', h, C, N, t_min, H);