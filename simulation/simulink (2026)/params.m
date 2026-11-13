mppt.Ts   = 2e-3;   % MPPT update period (s)
mppt.Du   = 0.005;  % duty step per update
mppt.D0   = 0.80;   % starting duty (0)
mppt.Dmin = 0.10;   % lower duty limit
mppt.Dmax = 0.95;   % upper duty limit
filt.a = 0.5;

Ts = 2e-6;

clear irr
irr.time = [0; 0.3; 0.6; 0.9];
irr.signals.values = [1000; 600; 300; 800];

fc = 100;             % cutoff in Hz
filt.a = exp(-2*pi*fc*Ts); 