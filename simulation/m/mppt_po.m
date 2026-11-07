function D = mppt_po(V, I, dD, D0, Dmin, Dmax)
% Perturb & Observe MPPT acting directly on the duty cycle.

persistent Dk Pprev dir
if isempty(Dk)                   % first call: initialize memory
    Dk = D0; Pprev = 0; dir = 1;
end

P = V * I;                       % present PV power
if P < Pprev                     % power dropped -> reverse direction
    dir = -dir;
end
Dk = Dk + dir * dD;              % take one step
Dk = min(max(Dk, Dmin), Dmax);   % stay within safe limits
Pprev = P;
D = Dk;
end