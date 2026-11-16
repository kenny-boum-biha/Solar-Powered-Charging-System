% PARAMS -- workspace variables used by Full_System.slx.
% Runs automatically before each simulation (InitFcn callback).

Ts = 1e-7;                        % Simulation / PWM time step, s.
                                  % PWM duty resolution = Ts / 20 us = 0.5 %.
                                  % If you change Ts, also change
                                  % Model Settings > Solver > Fixed-step size.

filt.tau = 5e-5;                  % Measurement low-pass time constant, s (3.2 kHz).
filt.a   = exp(-Ts/filt.tau);     % Filter coefficient used by the Discrete Transfer Fcn blocks.
