close all
clear
clc

addpath(data)
%Initialise helicopter constants
init_heli_3_10

V_d0 = -1.35;
V_s0 = 7.07;

K_f = - (m_c*g*l_c-2*m_p*g*l_h)/(V_s0*l_h);
L_1 = K_f*l_p;
L_2 = m_c*g*l_c-2*m_p*g*l_h;
L_3 = K_f*l_h;
L_4 = -L_3;

J_p = 2*m_p*l_p^2;
J_e = m_c*l_c^2 + 2*m_p*l_h^2;
J_l = m_c*l_c^2 + 2*m_p*(l_h^2 + l_p^2);

K_1 = L_1/J_p;
K_2 = L_3/J_e;
K_3 = L_4*V_s0;

%Part 1 task 3
lambda_1 = 1;
lambda_2 = 1;

K_pp = lambda_1 * lambda_2 / K_1;
K_pd = -(lambda_1 + lambda_2);