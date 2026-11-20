function [boostDutyCycle, buckDutyCycle] = solar_charge_controller(pvVoltage, pvCurrent, batteryVoltage, batteryCurrent, loadVoltage, chargingEnabled, loadEnabled, resetController)

%#codegen
% Calculate two duty ratios (0 to 1) from measured volts and amperes.
% batteryCurrent must be POSITIVE when current flows INTO the battery.
% Call this function every 10 us. Separate PWM blocks generate the pulses.
% pvVoltage, pvCurrent and batteryCurrent must be low-pass filtered before
% this block. batteryVoltage and loadVoltage are averaged here over one
% switching period (two calls), which removes the 50 kHz switching ripple.
% Enable and reset inputs are commands (0 or 1), not extra measurements.
%
% BOOST (PV -> battery): open-circuit start, MPPT (perturb and observe),
%                        battery current limit, battery voltage limit.
% BUCK  (battery -> 5 V): soft start, PID voltage regulation,
%                        low-battery cutoff, output over-voltage cutoff.

%% SETTINGS
controlPeriod = 10e-6;            % Seconds between controller executions.

% --- MPPT ---
mpptUpdateSamples = 500;          % 500 calls = 5 ms between MPPT steps. Range 200-1000.
                                  % Must be longer than the boost settling time (~2-3 ms).

mpptDutyStep = 0.0005;             % Duty change per MPPT step. Range 0.0025-0.01.
                                  % Must be >= PWM resolution (simulation step / 20 us).

openCircuitSettleSamples = 500;   % 5 ms with the boost off before each start, so the
                                  % PV voltage reading is the true open-circuit voltage.

maximumPowerVoltageFraction = 0.85; % First guess: Vmp = 0.85 x Voc (panel: 24/28 = 0.857).
                                  % Range 0.76-0.86 for silicon panels.

boostDutySlewRate = 100;          % Maximum duty change per second (0 -> 0.5 in 5 ms).
                                  % Soft start: limits inductor current surge. Range 20-200.

maximumBoostDuty = 0.75;          % Duty limit from the TI sizing equations.

% --- PV and battery limits ---
minimumPvVoltage = 15;            % Below this the panel is dark or disconnected, V.

maximumPvVoltage = 34;            % Above cold open-circuit voltage (32.5 V at -10 C), V.

minimumMpptVoltage = 18.3;        % MPPT may not pull the panel below this, V.

minimumChargingVoltage = 42;      % No deeply discharged battery recovery, V.

chargingVoltageTarget = 54;       % Renogy battery charging target, V.

maximumChargingCurrent = 10;      % Project converter limit, NOT battery rating, A.

limiterMargin = 0.05;             % Battery limiters wait this far above the MPPT duty,
                                  % so they take over quickly when a limit is reached.

% --- 5 V output ---
loadVoltageTarget = 5;            % USB supply target, V.

loadOvervoltageLimit = 6;         % Buck switch is held off above this output voltage, V.

maximumBuckDuty = 0.25;           % About 2 x the normal duty (5 V / 42-55.5 V = 0.09-0.12).

loadShutdownVoltage = 42;         % Low-battery load cutoff, V.

loadRestartVoltage = 47.3;        % Restart threshold avoids on/off chatter, V.

buckStartupTime = 0.020;          % Ramp the buck target to 5 V over 20 ms.

buckDerivativeFilterTime = 16e-6; % Smooths the voltage-slope estimate (10 kHz), s.

% --- Controller gains ---
% Proportional gains convert present error to duty correction.
% Integral gains remove sustained error by accumulating correction over time.
% The derivative gain reacts to how fast the output voltage is moving; it damps
% the buck L-C resonance so the loop can respond quickly to load changes.
% Values below are tuned for: boost L = 220 uH; buck L = 100 uH, C = 470 uF;
% 50 kHz switching. Retune if those parts change (see ranges in the summary).

chargeCurrentProportionalGain = 0.001;  % Duty/A.       Range 0.0005-0.002.

chargeCurrentIntegralGain = 2;          % Duty/(A*s).   Range 0.5-4.

chargeVoltageProportionalGain = 0.005;  % Duty/V.       Range 0.002-0.01.

chargeVoltageIntegralGain = 5;          % Duty/(V*s).   Range 2-10.

buckVoltageProportionalGain = 0.05;     % Duty/V.       Range 0.025-0.075.

buckVoltageIntegralGain = 60;           % Duty/(V*s).   Range 25-115.

buckVoltageDerivativeGain = 1.1e-5;     % Duty/(V/s).   Range 0.8e-5 to 1.4e-5.

assert(all(isfinite([chargeCurrentProportionalGain, chargeCurrentIntegralGain, chargeVoltageProportionalGain, chargeVoltageIntegralGain, buckVoltageProportionalGain, buckVoltageIntegralGain, buckVoltageDerivativeGain])), 'Set the seven controller gains before running the controller.');

%% MEMORY -- persistent values survive between calls to this function.

persistent mpptDutyRequest appliedBoostDuty previousPvPower perturbationDirection mpptSampleCount

persistent previousPowerIsValid powerSum powerSampleCount boostRunning offSampleCount

persistent currentLimitIntegral voltageLimitIntegral

persistent buckVoltageIntegral loadAllowed startupFraction

persistent previousBatteryVoltage previousLoadVoltage previousAveragedLoadVoltage loadVoltageSlope

resetRequested = (resetController ~= 0);

invalidMeasurements = ~all(isfinite([pvVoltage, pvCurrent, batteryVoltage, batteryCurrent, loadVoltage])) || batteryVoltage <= 0;

if isempty(mpptDutyRequest) || resetRequested || invalidMeasurements

    mpptDutyRequest = 0; appliedBoostDuty = 0;

    previousPvPower = 0; perturbationDirection = 1; mpptSampleCount = 0;

    previousPowerIsValid = false; powerSum = 0; powerSampleCount = 0;

    boostRunning = false; offSampleCount = 0;

    currentLimitIntegral = maximumBoostDuty;

    voltageLimitIntegral = maximumBoostDuty;

    buckVoltageIntegral = 0; loadAllowed = false; startupFraction = 0;

    previousBatteryVoltage = batteryVoltage; previousLoadVoltage = loadVoltage;

    previousAveragedLoadVoltage = loadVoltage; loadVoltageSlope = 0;

    if invalidMeasurements          % Never store NaN or Inf in memory.

        previousBatteryVoltage = 0; previousLoadVoltage = 0; previousAveragedLoadVoltage = 0;

    end

end

boostDutyCycle = 0; buckDutyCycle = 0;

if resetRequested || invalidMeasurements

    return                      % Both switches remain off for this call.

end

%% MEASUREMENTS -- average two consecutive samples (= one 20 us switching period).
% The controller samples twice per switching period, so this average cancels
% the switching ripple instead of letting it disturb the control loops.

averagedBatteryVoltage = 0.5*(batteryVoltage + previousBatteryVoltage);

averagedLoadVoltage = 0.5*(loadVoltage + previousLoadVoltage);

previousBatteryVoltage = batteryVoltage; previousLoadVoltage = loadVoltage;

% Filtered rate of change of the output voltage, V/s (used by the buck PID).

rawSlope = (averagedLoadVoltage - previousAveragedLoadVoltage)/controlPeriod;

loadVoltageSlope = loadVoltageSlope + controlPeriod/(buckDerivativeFilterTime + controlPeriod)*(rawSlope - loadVoltageSlope);

previousAveragedLoadVoltage = averagedLoadVoltage;

%% BUCK -- maintain the USB output at 5 V while the battery can supply it.

if (loadEnabled == 0) || averagedBatteryVoltage <= loadShutdownVoltage

    loadAllowed = false;

elseif averagedBatteryVoltage >= loadRestartVoltage

    loadAllowed = true;

end                             % Between thresholds, retain the previous state.

if loadAllowed && averagedLoadVoltage <= loadOvervoltageLimit

    startupFraction = min(startupFraction + controlPeriod/buckStartupTime, 1);

    rampedLoadTarget = startupFraction*loadVoltageTarget;

    loadVoltageError = rampedLoadTarget - averagedLoadVoltage;

    % Ideal duty = output/input voltage; PID feedback corrects the actual error.

    requestedBuckDuty = rampedLoadTarget/averagedBatteryVoltage + buckVoltageProportionalGain*loadVoltageError - buckVoltageDerivativeGain*loadVoltageSlope + buckVoltageIntegral;

    buckDutyCycle = min(max(requestedBuckDuty, 0), maximumBuckDuty);

    % Anti-windup: do not accumulate error that pushes beyond a duty limit.

    if (requestedBuckDuty >= 0 && requestedBuckDuty <= maximumBuckDuty) || (requestedBuckDuty > maximumBuckDuty && loadVoltageError < 0) || (requestedBuckDuty < 0 && loadVoltageError > 0)

        buckVoltageIntegral = buckVoltageIntegral + buckVoltageIntegralGain*controlPeriod*loadVoltageError;

    end

elseif ~loadAllowed

    buckVoltageIntegral = 0; startupFraction = 0;

end                             % Over-voltage: switch stays off, memory is kept.

%% BOOST -- permit charging only when the panel and battery are in range.

chargingPermitted = (chargingEnabled ~= 0) && pvVoltage > minimumPvVoltage && pvVoltage < maximumPvVoltage && averagedBatteryVoltage >= minimumChargingVoltage && pvVoltage < averagedBatteryVoltage;

if ~chargingPermitted

    boostRunning = false; offSampleCount = 0;

    mpptDutyRequest = 0; appliedBoostDuty = 0;

    previousPowerIsValid = false; mpptSampleCount = 0; powerSum = 0; powerSampleCount = 0;

    currentLimitIntegral = maximumBoostDuty;

    voltageLimitIntegral = maximumBoostDuty;

    return                      % Boost stays off; buck may still supply the load.

end

if ~boostRunning

    % Boost is off, so the panel is unloaded and its voltage rises to Voc.

    offSampleCount = offSampleCount + 1;

    if offSampleCount < openCircuitSettleSamples

        return                  % Keep waiting for the open-circuit reading.

    end

    % Start near the maximum power point: Vmp is about 0.85 x Voc, and for a
    % boost converter PV voltage = battery voltage x (1 - duty).

    mpptDutyRequest = min(max(1 - maximumPowerVoltageFraction*pvVoltage/averagedBatteryVoltage, 0), maximumBoostDuty);

    boostRunning = true;

    previousPowerIsValid = false; mpptSampleCount = 0; perturbationDirection = 1;

    powerSum = 0; powerSampleCount = 0;

end

% Each battery PI loop sets a ceiling on boost duty. The lower ceiling wins.

chargingCurrentError = maximumChargingCurrent - batteryCurrent;

chargingVoltageError = chargingVoltageTarget - averagedBatteryVoltage;

currentLimitedDuty = min(max(currentLimitIntegral + chargeCurrentProportionalGain*chargingCurrentError, 0), maximumBoostDuty);

voltageLimitedDuty = min(max(voltageLimitIntegral + chargeVoltageProportionalGain*chargingVoltageError, 0), maximumBoostDuty);

batteryDutyCeiling = min(currentLimitedDuty, voltageLimitedDuty);

%% MPPT -- perturb boost duty slowly and observe the change in PV power.

pvPower = pvVoltage*pvCurrent;

slewStep = boostDutySlewRate*controlPeriod;   % Largest duty change in one call.

if batteryDutyCeiling < mpptDutyRequest

    % Battery regulation takes priority. Restart power comparisons afterward.

    mpptDutyRequest = batteryDutyCeiling;

    previousPowerIsValid = false; mpptSampleCount = 0; perturbationDirection = 1;

    powerSum = 0; powerSampleCount = 0;

elseif abs(appliedBoostDuty - mpptDutyRequest) > slewStep

    % Duty is still ramping to its target: wait before judging the power.

    mpptSampleCount = 0; powerSum = 0; powerSampleCount = 0;

else

    mpptSampleCount = mpptSampleCount + 1;

    % Average the power over the second half of the interval, after the
    % converter has settled. An average is far less noisy than one sample.

    if 2*mpptSampleCount > mpptUpdateSamples

        powerSum = powerSum + pvPower; powerSampleCount = powerSampleCount + 1;

    end

    if mpptSampleCount >= mpptUpdateSamples

        averagePvPower = powerSum/max(powerSampleCount, 1);

        if previousPowerIsValid

            if averagePvPower < previousPvPower

                perturbationDirection = -perturbationDirection;

            end                 % Power decreased: reverse the next duty step.

            if pvVoltage < minimumMpptVoltage

                perturbationDirection = -1;

            end                 % PV voltage too low: lower duty to raise it.

            mpptDutyRequest = min(max(mpptDutyRequest + perturbationDirection*mpptDutyStep, 0), maximumBoostDuty);

        end                     % First observation establishes the reference.

        previousPvPower = averagePvPower; previousPowerIsValid = true;

        mpptSampleCount = 0; powerSum = 0; powerSampleCount = 0;

    end

end

% Move the applied duty toward its target at a limited rate (soft start).

boostDutyTarget = min(mpptDutyRequest, batteryDutyCeiling);

appliedBoostDuty = appliedBoostDuty + min(max(boostDutyTarget - appliedBoostDuty, -slewStep), slewStep);

boostDutyCycle = appliedBoostDuty;

%% UPDATE BATTERY PI MEMORY -- avoid accumulating beyond the duty limits.

if (currentLimitedDuty > 0 && currentLimitedDuty < maximumBoostDuty) || (currentLimitedDuty == 0 && chargingCurrentError > 0) || (currentLimitedDuty == maximumBoostDuty && chargingCurrentError < 0)

    currentLimitIntegral = currentLimitIntegral + chargeCurrentIntegralGain*controlPeriod*chargingCurrentError;

end

if (voltageLimitedDuty > 0 && voltageLimitedDuty < maximumBoostDuty) || (voltageLimitedDuty == 0 && chargingVoltageError > 0) || (voltageLimitedDuty == maximumBoostDuty && chargingVoltageError < 0)

    voltageLimitIntegral = voltageLimitIntegral + chargeVoltageIntegralGain*controlPeriod*chargingVoltageError;

end

% Keep idle limiters just above the MPPT duty so they engage without delay.

currentLimitIntegral = min(currentLimitIntegral, mpptDutyRequest + limiterMargin);

voltageLimitIntegral = min(voltageLimitIntegral, mpptDutyRequest + limiterMargin);

end
