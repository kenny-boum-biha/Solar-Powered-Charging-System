%% 1. PARAMETERS -- edit this section, then Run the entire file
% SIZE_BOOST_CONVERTER: PV panel to battery.
% MATLAB R2024a; base MATLAB only. Does NOT create/change a Simulink model.
% Source: TI SLVA372D, equations (1)-(13).
% https://www.ti.com/lit/an/slva372d/slva372d.pdf
% Continuous-conduction, nonsynchronous power-stage estimates, not control tuning.
% NaN means unspecified/automatic, as explained next to each parameter.
% Results remain in workspace struct boost_results; no files are overwritten.

clear boost_results % avoid displaying stale results after a failed calculation
p = struct();
% Battery: Renogy Core RBT4850LFPSH, 48 V / 50 Ah (15-cell LiFePO4).
% Source: https://ca.renogy.com/content/manual/RBT4850LFPSH_UM.pdf
% Manual pp. 10-14: range 42-55.5 V, charge 54 V, max continuous charge/discharge 50 A.
% ASSERT comments identify unconfirmed design choices, not MATLAB assert() calls.
p.battery_capacity_Ah = 50; % Reference only; capacity does not set converter current.
p.battery_charge_limit_A = 50; % A; manufacturer maximum continuous charge current.
% PV source: Arch_16BB_N-Type_User_Manual.pdf, printed p. 7 (PDF page 11).
p.panel_model_W = 200; % W; user-confirmed Arch 200 panel (100 option also supported).
p.panels_series = 1;   % User-confirmed: one panel.
p.strings_parallel = 1;% One panel total; no parallel strings added.
switch p.panel_model_W
    case 100
        p.panel_Vmp_STC = 25.2; p.panel_Imp_STC = 3.97;
        p.panel_Voc_STC = 29.4; p.panel_Isc_STC = 4.17;
    case 200
        p.panel_Vmp_STC = 24; p.panel_Imp_STC = 8.34;
        p.panel_Voc_STC = 28; p.panel_Isc_STC = 8.75;
    otherwise
        error('Select panel_model_W = 100 or 200 from the supplied manual.');
end
% Manual electrical ratings have +/-5% tolerance; voltages V, currents A.
p.PV_rating_tolerance = 0.05;
p.Voc_temp_coefficient = -0.003; % /degC; manual -0.3 percent/K.
p.cell_temperature_min_C = -10; % ASSERT: determine site minimum CELL temperature.
% -10 C is the published panel operating lower limit, not a verified site minimum.
p.Vin_nom = p.panels_series*p.panel_Vmp_STC;
p.Vin_min = 0.80*p.Vin_nom; % ASSERT: determine hot/low-light MPPT minimum; placeholder.
p.Vin_max = 1.15*p.Vin_nom; % ASSERT: determine cold MPPT maximum; placeholder.
% The manual does not give a Vmp temperature coefficient; do not substitute Voc's.
p.Vin_stress_max = p.panels_series*p.panel_Voc_STC*(1+p.PV_rating_tolerance)* ...
    (1+p.Voc_temp_coefficient*(p.cell_temperature_min_C-25)); % ASSERT: add transients.
p.Vout_min = 42;      % V; ASSERT: confirm lowest battery voltage at charge enable.
p.Vout_nom = 54;      % V; manufacturer CC/CV charge target (54 +/-0.2 V).
p.Vout_max = 54.2;    % V; upper charge-setting tolerance; not the battery's 55.5 V limit.
p.Vout_stress_max = 55.5; % V; ASSERT: confirm battery/BMS-disconnect transient maximum.
p.Vout_ripple_pp = 0.10; % V p-p; ASSERT: allocate ripple, sensing error and overshoot within charge tolerance.
p.Cout_ESR = 0.005;   % ohm; ASSERT: select capacitor bank and verify worst-case ESR.
p.eta = 0.90;             % ASSERT: determine efficiency across the operating range.
p.fsw = 20e3;             % Hz; ASSERT: choose minimum switching frequency.
p.ripple_fraction = 0.30; % ASSERT: confirm allowed inductor ripple fraction.
p.L_tolerance = 0.20;     % ASSERT: determine negative tolerance and saturation derating.
p.C_tolerance = 0.20;     % ASSERT: determine capacitance loss from tolerance, bias, temperature.
p.L_selected_H = NaN;     % ASSERT: select a real inductor; NaN suggests an E12 value.
p.Cout_selected_F = NaN;  % ASSERT: select a real capacitor bank; NaN suggests E12.
p.diode_Vf = 0.8;         % V; ASSERT: determine diode drop at operating current.
p.current_margin = 1.25;  % ASSERT: confirm engineering current margin (not TI equation).
p.voltage_margin = 1.25;  % ASSERT: confirm engineering voltage margin (not TI equation).
p.IC_current_limit_min_A = NaN; % ASSERT: guaranteed minimum PEAK switch limit, selected IC.
p.Cin_datasheet_min_F = NaN;    % ASSERT: input capacitance from controller datasheet (TI sec. 6).
p.Vfb = NaN;             % ASSERT: decide if analog divider is used; enter reference V if so.
p.Ifb_max_A = NaN;       % ASSERT: feedback-pin bias current A, if analog divider is used.
p.Idivider_A = 100e-6;   % ASSERT: choose divider current >=100*Ifb_max_A, if used.
p.PV_available_W = p.panel_model_W*p.panels_series*p.strings_parallel* ...
    (1+p.PV_rating_tolerance); % ASSERT: confirm max power incl. cold/high irradiance; STC +5% only.
p.Iout_max = min(p.battery_charge_limit_A,p.eta*p.PV_available_W/p.Vout_max);
% ASSERT: confirm this CONSTANT charge-current cap in the controller design.
% This cap deliberately limits current even at low battery voltage. If MPPT is
% allowed to deliver more current there, raise Iout_max and re-size the stage.
% PV_available_W is a sizing power envelope, not guaranteed sunlight or charge power.

%% 2. EQUATIONS -- TI equation numbers are cited beside each calculation
% Extra range/tolerance checks and E12 rounding are explicitly engineering additions.
validate_inputs(p);
assert(all([p.panels_series p.strings_parallel]>=1) && ...
    all(mod([p.panels_series p.strings_parallel],1)==0), 'Panel counts must be positive integers.');
assert(isfinite(p.Vout_stress_max) && p.Vout_stress_max>=p.Vout_max, 'Output stress must cover maximum output voltage.');
assert(p.Iout_max<=p.battery_charge_limit_A, 'Charge current exceeds battery continuous limit.');
assert(p.Vin_stress_max<p.Vout_min, ...
    'PV open-circuit voltage can exceed minimum battery voltage: review boost-only topology and reverse-current isolation.');
r = struct();
r.parameters = p;
assert(p.Vout_min > p.Vin_max, 'Boost requires all output voltages above all operating input voltages.');
assert(isfinite(p.PV_available_W) && p.PV_available_W>0, 'PV available power must be positive.');
% TI eq. 6 includes Vout/Vin: ripple fraction refers approximately to INPUT current.
r.target_ripple_A = p.ripple_fraction*p.Iout_max*p.Vout_nom/p.Vin_nom;
% TI eq. 5: ideal nominal-point initial inductance estimate.
r.L_TI_nominal_H = p.Vin_nom*(p.Vout_nom-p.Vin_nom) / ...
    (r.target_ripple_A*p.fsw*p.Vout_nom);
% Range extension: ripple can peak INSIDE the input range, at Vin=Vout/(2*eta).
vo = linspace(p.Vout_min,p.Vout_max,101);
vi = unique([linspace(p.Vin_min,p.Vin_max,101), ...
    min(p.Vin_max,max(p.Vin_min,vo/(2*p.eta)))]);
[VI,VO] = ndgrid(vi,vo);
D = 1-VI*p.eta./VO;                         % TI eq. 1, estimated duty
assert(all(D(:)>0 & D(:)<1), 'TI duty estimate outside (0,1).');
% TI eq. 2 inverted, retaining a fixed nominal-point ripple target across the range.
Lrequired = VI.*D/(p.fsw*r.target_ripple_A);
r.L_effective_required_H = max(Lrequired(:));
r.L_nominal_required_H = r.L_effective_required_H/(1-p.L_tolerance);
if isnan(p.L_selected_H)
    r.L_selected_H = e12_up(r.L_nominal_required_H);
else
    r.L_selected_H = p.L_selected_H;
end
Lmin = r.L_selected_H*(1-p.L_tolerance);
dIL = VI.*D/(p.fsw*Lmin); % TI eq. 2, selected L at low tolerance
ILavg = p.Iout_max./(1-D);
Ipeak = ILavg+dIL/2;                         % TI eq. 4
r.duty_min = min(D(:)); r.duty_max = max(D(:));
r.inductor_ripple_max_A = max(dIL(:));
r.inductor_peak_A = max(Ipeak(:));
r.inductor_RMS_A = max(sqrt(ILavg(:).^2+dIL(:).^2/12)); % triangular-current extension
r.inductor_saturation_rating_A = p.current_margin*r.inductor_peak_A;
r.inductor_thermal_rating_A = p.current_margin*r.inductor_RMS_A;
r.switch_peak_rating_A = p.current_margin*r.inductor_peak_A;
r.ripple_target_met = r.inductor_ripple_max_A <= r.target_ripple_A*(1+1e-10);
r.CCM_at_design_load = all(ILavg(:)-dIL(:)/2>0);
% Light-load operation can enter DCM even when full-load CCM passes.
Idiode = p.Iout_max*ones(size(D));            % TI eq. 7
VrESR = p.Cout_ESR*(p.Iout_max./(1-D)+dIL/2); % TI eq. 13
capNumerator = p.Iout_max*D/p.fsw;            % TI eq. 12 numerator
r.diode_average_A = max(Idiode(:));
r.diode_average_rating_A = p.current_margin*r.diode_average_A;
r.diode_peak_rating_A = p.current_margin*r.inductor_peak_A;
r.diode_conduction_loss_W = p.diode_Vf*r.diode_average_A; % TI eq. 8; excludes switching
% Do not allocate the ENTIRE ripple budget to capacitance and add ESR afterward.
remainingRipple = p.Vout_ripple_pp-VrESR;
assert(all(remainingRipple(:)>0), ...
    'ESR alone consumes the output ripple budget. Reduce Cout_ESR or relax Vout_ripple_pp.');
Crequired = capNumerator./remainingRipple;    % TI eq. 12 + 13, combined budget
r.Cout_effective_required_F = max(Crequired(:));
r.Cout_nominal_required_F = r.Cout_effective_required_F/(1-p.C_tolerance);
if isnan(p.Cout_selected_F)
    r.Cout_selected_F = e12_up(r.Cout_nominal_required_F);
else
    r.Cout_selected_F = p.Cout_selected_F;
end
Cmin = r.Cout_selected_F*(1-p.C_tolerance);
r.output_ripple_estimate_Vpp = max(capNumerator(:)/Cmin+VrESR(:));
r.output_ripple_target_met = r.output_ripple_estimate_Vpp <= p.Vout_ripple_pp*(1+1e-10);
% Voltage ratings: topology blocking stress plus user margin; not TI eq. 1-13.
stress = max(p.Vin_stress_max,p.Vout_stress_max);
r.switch_voltage_rating_V = p.voltage_margin*stress;
r.diode_reverse_rating_V = p.voltage_margin*stress;
r.Cout_voltage_rating_V = p.voltage_margin*p.Vout_stress_max;
r.Cin_voltage_rating_V = p.voltage_margin*p.Vin_stress_max;
r.output_power_max_W = p.Vout_max*p.Iout_max;
r.input_power_required_W = r.output_power_max_W/p.eta;
r.input_current_max_A = p.Vout_max*p.Iout_max/(p.eta*p.Vin_min);
r.Cin_datasheet_min_F = p.Cin_datasheet_min_F; % TI sec. 6: no universal sizing formula
r.IC_output_capability_A = NaN;
if ~isnan(p.IC_current_limit_min_A)
    capability = (p.IC_current_limit_min_A-dIL/2).*(1-D); % TI eq. 3
    r.IC_output_capability_A = min(capability(:));
    if r.IC_output_capability_A < p.Iout_max
        warning('Selected controller switch-current limit is insufficient at one or more operating points.');
    end
end
% TI eqs. 9-11: optional nominal-voltage analog feedback divider.
r.Rtop_ohm = NaN; r.Rbottom_ohm = NaN;
if ~isnan(p.Vfb)
    assert(isfinite(p.Vfb) && p.Vfb>0 && p.Vfb<p.Vout_nom, 'Vfb must be between 0 and Vout_nom.');
    assert(isfinite(p.Ifb_max_A) && p.Ifb_max_A>=0, 'Enter the maximum feedback pin bias current.');
    assert(p.Idivider_A>=100*p.Ifb_max_A && p.Idivider_A>0, 'Divider current must be >=100*Ifb_max_A.');
    r.Rbottom_ohm = p.Vfb/p.Idivider_A;         % TI eq. 10
    r.Rtop_ohm = r.Rbottom_ohm*(p.Vout_nom/p.Vfb-1); % TI eq. 11
end
if ~r.ripple_target_met, warning('Selected inductance does not meet the current-ripple target.'); end
if ~r.output_ripple_target_met, warning('Selected capacitance does not meet the output-ripple target.'); end
if ~r.CCM_at_design_load, warning('CCM assumption fails at the design load; these equations are not valid there.'); end
if r.input_power_required_W > p.PV_available_W*(1+1e-10)
    warning('Requested charge power exceeds the entered available PV power after losses.');
end

%% 3. RESULTS -- nominal component choices, required ratings, and checks
boost_results = r;
fprintf('\n=== BOOST CONVERTER: TI-BASED PRELIMINARY SIZING ===\n');
fprintf('Input %.3f to %.3f V; output %.3f to %.3f V; load %.3f A\n', ...
    p.Vin_min,p.Vin_max,p.Vout_min,p.Vout_max,p.Iout_max);
fprintf('Duty estimate: %.2f to %.2f %%\n',100*r.duty_min,100*r.duty_max);
fprintf('TI nominal-point L (eq. 5, before margins): %.2f uH\n',1e6*r.L_TI_nominal_H);
fprintf('L nominal minimum, range + tolerance:      %.2f uH\n',1e6*r.L_nominal_required_H);
fprintf('L selected nominal:                       %.2f uH\n',1e6*r.L_selected_H);
fprintf('Inductor ripple: %.3f A p-p; target: %.3f A p-p\n',r.inductor_ripple_max_A,r.target_ripple_A);
fprintf('Inductor peak: %.3f A; RMS: %.3f A\n',r.inductor_peak_A,r.inductor_RMS_A);
fprintf('Inductor saturation / thermal ratings: >= %.3f / %.3f A (with margin)\n', ...
    r.inductor_saturation_rating_A,r.inductor_thermal_rating_A);
fprintf('Cout nominal minimum, ESR + tolerance:     %.2f uF\n',1e6*r.Cout_nominal_required_F);
fprintf('Cout selected nominal:                    %.2f uF\n',1e6*r.Cout_selected_F);
fprintf('Estimated output ripple: %.4f V p-p; budget: %.4f V p-p\n', ...
    r.output_ripple_estimate_Vpp,p.Vout_ripple_pp);
fprintf('Cout / Cin minimum voltage ratings: %.2f / %.2f V (with margin)\n', ...
    r.Cout_voltage_rating_V,r.Cin_voltage_rating_V);
fprintf('Switch minimum voltage / peak current ratings: %.2f V / %.3f A\n', ...
    r.switch_voltage_rating_V,r.switch_peak_rating_A);
fprintf('Diode minimum reverse / average / peak ratings: %.2f V / %.3f A / %.3f A\n', ...
    r.diode_reverse_rating_V,r.diode_average_rating_A,r.diode_peak_rating_A);
fprintf('Diode conduction loss estimate: %.3f W\n',r.diode_conduction_loss_W);
fprintf('Output power: %.2f W; required input power: %.2f W\n', ...
    r.output_power_max_W,r.input_power_required_W);
fprintf('Checks (1=pass): L ripple=%d; output ripple=%d; full-load CCM=%d\n', ...
    r.ripple_target_met,r.output_ripple_target_met,r.CCM_at_design_load);
if isnan(r.Cin_datasheet_min_F)
    fprintf('Cin capacitance: NOT SIZED. Enter controller datasheet minimum (TI section 6).\n');
else
    fprintf('Cin datasheet minimum (effective capacitance): %.2f uF\n',1e6*r.Cin_datasheet_min_F);
end
if isnan(r.IC_output_capability_A)
    fprintf('Controller current-limit check: NOT CHECKED; no IC selected.\n');
else
    fprintf('Controller worst-case output capability: %.3f A\n',r.IC_output_capability_A);
end
if ~isnan(r.Rtop_ohm)
    fprintf('Feedback divider: Rtop=%.2f ohm; Rbottom=%.2f ohm (unrounded).\n', ...
        r.Rtop_ohm,r.Rbottom_ohm);
end
fprintf('Full results saved in workspace: boost_results\n');
fprintf('E12 values are suggestions, not selected parts. Check datasheets, losses,\n');
fprintf('capacitor ripple-current ratings, transients, and control-loop stability.\n');

%% Local helpers -- normally leave these unchanged
function value = e12_up(required)
% Engineering addition: round UP to a preferred nominal value, not an exact part.
e12 = [1 1.2 1.5 1.8 2.2 2.7 3.3 3.9 4.7 5.6 6.8 8.2 10];
scale = 10^floor(log10(required));
value = scale*e12(find(e12*scale>=required,1,'first'));
end

function validate_inputs(p)
positive = {'Vin_min','Vin_nom','Vin_max','Vout_min','Vout_nom','Vout_max', ...
    'fsw','Iout_max','Vout_ripple_pp','Vin_stress_max','Idivider_A'};
for k=1:numel(positive)
    x=p.(positive{k});
    assert(isnumeric(x) && isscalar(x) && isfinite(x) && x>0, ...
        '%s must be a positive finite scalar.',positive{k});
end
assert(p.Vin_min<=p.Vin_nom && p.Vin_nom<=p.Vin_max, 'Vin_min <= Vin_nom <= Vin_max required.');
assert(p.Vout_min<=p.Vout_nom && p.Vout_nom<=p.Vout_max, 'Vout_min <= Vout_nom <= Vout_max required.');
assert(isscalar(p.eta) && isfinite(p.eta) && p.eta>0 && p.eta<=1, 'Efficiency must be in (0,1].');
assert(isscalar(p.ripple_fraction) && isfinite(p.ripple_fraction) && p.ripple_fraction>0 && p.ripple_fraction<2, 'Invalid ripple fraction.');
for field={'L_tolerance','C_tolerance'}
    x=p.(field{1}); assert(isscalar(x) && isfinite(x) && x>=0 && x<1, 'Tolerances must be in [0,1).');
end
assert(isfinite(p.Cout_ESR) && p.Cout_ESR>=0 && isscalar(p.Cout_ESR), 'ESR must be nonnegative.');
assert(isfinite(p.diode_Vf) && p.diode_Vf>=0 && isscalar(p.diode_Vf), 'Diode drop must be nonnegative.');
assert(isfinite(p.current_margin) && p.current_margin>=1 && isscalar(p.current_margin), 'Current margin must be >=1.');
assert(isfinite(p.voltage_margin) && p.voltage_margin>=1 && isscalar(p.voltage_margin), 'Voltage margin must be >=1.');
assert(isnumeric(p.Vfb) && isscalar(p.Vfb) && (isnan(p.Vfb) || (isfinite(p.Vfb) && p.Vfb>0)), 'Vfb must be NaN or a positive finite scalar.');
assert(isnumeric(p.Ifb_max_A) && isscalar(p.Ifb_max_A) && (isnan(p.Ifb_max_A) || (isfinite(p.Ifb_max_A) && p.Ifb_max_A>=0)), 'Feedback bias current must be NaN or nonnegative.');
assert(p.Vin_stress_max>=p.Vin_max, 'Input stress voltage cannot be below operating maximum.');
for field={'L_selected_H','Cout_selected_F','IC_current_limit_min_A','Cin_datasheet_min_F'}
    x=p.(field{1}); assert(isnumeric(x) && isscalar(x) && (isnan(x) || (isfinite(x) && x>0)), ...
        '%s must be NaN or a positive finite scalar.',field{1});
end
end
