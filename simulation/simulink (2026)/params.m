% PARAMS -- workspace variables used by Full_System.slx.
% Runs automatically before each simulation (InitFcn callback).

Ts = 1e-8;                        % Simulation / PWM time step, s.
                                  % PWM duty resolution = Ts / 20 us = 0.5 %.
                                  % If you change Ts, also change
                                  % Model Settings > Solver > Fixed-step size.

filt.tau = 5e-5;                  % Measurement low-pass time constant, s (3.2 kHz).
filt.a   = exp(-Ts/filt.tau);     % Filter coefficient used by the Discrete Transfer Fcn blocks.

% Expected maximum power point at 25 degC (from the PV block's I-V plot)
pv.G  = [400     600     800     1000];   % Irradiance, W/m^2
pv.Vm = [23.300  23.603  23.900  23.980]; % Voltage at maximum power, V
pv.Im = [3.323   5.001   6.653   8.346];  % Current at maximum power, A
pv.Pm = [77.437  118.03  159.00  200.13]; % Maximum power, W
