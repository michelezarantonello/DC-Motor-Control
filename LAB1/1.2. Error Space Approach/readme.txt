To run this project properly:

> Open a MATLAB Session in the directory where you saved this project in your computer

> Open final_results.m and RobErrorSpaceSSM.slx

> Run final_results.m

# What I am trying to realize:

% This script automates testing of an error-space controller for a black-box DC motor model in Simulink.
% It performs:
%   1. Sinusoidal period variation tests (controller redesigned each time)
%   2. Amplitude sweep at T_r = 0.5 s (controller designed for 0.5 s only)
%   3. Cross-test at T_r = 0.1 s with controller designed at T_r = 0.5 s
%   4. Step reference tracking test
%
% Results (RMS error, max error, steady-state RMS error, step metrics) are collected and plotted.


#Problems:

1. 
-> ss_error increases as period of sinusoidals increases (logically it should decrease)
-> settling time is 0 or NaN for all trials

2. Need to make a sanity check of these results (do they make sense?)

3. Need to make a sanity check of these results (do they make sense?)

4. Step reference test seems okay
