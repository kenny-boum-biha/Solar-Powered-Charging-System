function [boostDutyCycle, buckDutyCycle] = solar_charge_controller(pvVoltage, pvCurrent, batteryVoltage, batteryCurrent, loadVoltage, chargingEnabled, loadEnabled, resetController)

%#codegen
% Calculate two duty ratios (0 to 1) from measured volts and amperes.
% batteryCurrent must be POSITIVE when current flows INTO the battery.
% Call this function every 10 us. Separate PWM blocks generate the pulses.
% The five measurements should be filtered/sampled before entering this block.
% Enable and reset inputs are Boolean commands, not extra measurements.
% Simulation template: gains require tuning after L and C are selected.

%% SETTINGS
controlPeriod = 10e-6;           % Seconds between controller executions.

mpptUpdateSamples = 1000;        % 1000 calls = 10 ms between MPPT updates.

mpptDutyStep = 0.001;            % Change duty by 0.1 percentage point per step.

initialBoostDuty = 0;            % Start with the boost switch off.

maximumBoostDuty = 0.75;         % duty limits with TI equations

maximumBuckDuty = 0.25;

minimumPvVoltage = 18.3;           % Project PV operating window, V.

maximumPvVoltage = 28.6;

minimumChargingVoltage = 42;    % No deeply discharged battery recovery, V.

chargingVoltageTarget = 54;     % Renogy battery charging target, V.

maximumChargingCurrent = 10;     % Project converter limit, NOT battery rating, A.

loadVoltageTarget = 5;           % USB supply target, V.

loadShutdownVoltage = 42;       % low-battery load cutoff, V.

loadRestartVoltage = 47.3;        % restart threshold avoids on/off chatter.

buckStartupTime = 0.020;         % Ramp the buck target to 5 V over 20 ms.

% Proportional gains convert present error to duty correction.
% Integral gains remove sustained error by accumulating correction over time.
% Replace NaN with tuned values; NaN intentionally prevents an untuned run.

chargeCurrentProportionalGain = NaN;  % Duty/A.

chargeCurrentIntegralGain = NaN;      % Duty/(A*s).

chargeVoltageProportionalGain = NaN;  % Duty/V.

chargeVoltageIntegralGain = NaN;      % Duty/(V*s).

buckVoltageProportionalGain = NaN;    % Duty/V.

buckVoltageIntegralGain = NaN;        % Duty/(V*s).

assert(all(isfinite([chargeCurrentProportionalGain, chargeCurrentIntegralGain, chargeVoltageProportionalGain, chargeVoltageIntegralGain, buckVoltageProportionalGain, buckVoltageIntegralGain])), 'Set the six controller gains before running the controller.');

%% MEMORY -- persistent values survive between calls to this function.

persistent mpptDutyRequest previousPvPower perturbationDirection mpptSampleCount

persistent previousPowerIsValid currentLimitIntegral voltageLimitIntegral

persistent buckVoltageIntegral loadAllowed startupFraction

invalidMeasurements = ~all(isfinite([pvVoltage, pvCurrent, batteryVoltage, batteryCurrent, loadVoltage])) || batteryVoltage <= 0;

if isempty(mpptDutyRequest) || resetController || invalidMeasurements
   
    mpptDutyRequest = initialBoostDuty;
   
    previousPvPower = 0; perturbationDirection = 1; mpptSampleCount = 0;
   
    previousPowerIsValid = false;
   
    currentLimitIntegral = maximumBoostDuty;
   
    voltageLimitIntegral = maximumBoostDuty;
   
    buckVoltageIntegral = 0; loadAllowed = false; startupFraction = 0;
end

boostDutyCycle = 0; buckDutyCycle = 0;

if resetController || invalidMeasurements
 
    return                      % Both switches remain off for this call.

end

%% BUCK -- maintain the USB output at 5 V while the battery can supply it.
if ~loadEnabled || batteryVoltage <= loadShutdownVoltage
    
    loadAllowed = false;

elseif batteryVoltage >= loadRestartVoltage
  
    loadAllowed = true;

end                             % Between thresholds, retain the previous state.

if loadAllowed
   
    startupFraction = min(startupFraction + controlPeriod/buckStartupTime, 1);
    
    rampedLoadTarget = startupFraction*loadVoltageTarget;
   
   
    loadVoltageError = rampedLoadTarget - loadVoltage;
    
    % Ideal duty = output/input voltage; PI feedback corrects the actual error.
    
    requestedBuckDuty = rampedLoadTarget/batteryVoltage + buckVoltageProportionalGain*loadVoltageError + buckVoltageIntegral;
    
    buckDutyCycle = min(max(requestedBuckDuty, 0), maximumBuckDuty);
    
    % Anti-windup: do not accumulate error that pushes beyond a duty limit.
    
    if (requestedBuckDuty >= 0 && requestedBuckDuty <= maximumBuckDuty) || (requestedBuckDuty > maximumBuckDuty && loadVoltageError < 0) || (requestedBuckDuty < 0 && loadVoltageError > 0)
        buckVoltageIntegral = buckVoltageIntegral + buckVoltageIntegralGain*controlPeriod*loadVoltageError;
    end

else
    buckVoltageIntegral = 0; startupFraction = 0;
end

%% BOOST -- permit charging only within the assumed operating window.

if ~chargingEnabled || pvVoltage < minimumPvVoltage || pvVoltage > maximumPvVoltage || batteryVoltage < minimumChargingVoltage || pvVoltage >= batteryVoltage
    
    mpptDutyRequest = initialBoostDuty;
    
    previousPowerIsValid = false; mpptSampleCount = 0;
    
    currentLimitIntegral = maximumBoostDuty;
    
    voltageLimitIntegral = maximumBoostDuty;
    
    return                      % Boost stays off; buck may still supply the load.

end

% Each battery PI loop sets a ceiling on boost duty. The lower ceiling wins.

chargingCurrentError = maximumChargingCurrent - batteryCurrent;

chargingVoltageError = chargingVoltageTarget - batteryVoltage;

currentLimitedDuty = min(max(currentLimitIntegral + chargeCurrentProportionalGain*chargingCurrentError, 0), maximumBoostDuty);

voltageLimitedDuty = min(max(voltageLimitIntegral + chargeVoltageProportionalGain*chargingVoltageError, 0), maximumBoostDuty);

batteryDutyCeiling = min(currentLimitedDuty, voltageLimitedDuty);

%% MPPT -- perturb boost duty slowly and observe the change in PV power.

pvPower = pvVoltage*pvCurrent;

mpptSampleCount = mpptSampleCount + 1;

if batteryDutyCeiling < mpptDutyRequest
    
    % Battery regulation takes priority. Restart power comparisons afterward.
    
    mpptDutyRequest = batteryDutyCeiling;
    
    previousPowerIsValid = false; mpptSampleCount = 0; perturbationDirection = 1;

elseif mpptSampleCount >= mpptUpdateSamples
    
    if previousPowerIsValid
        
        if pvPower < previousPvPower
           
            perturbationDirection = -perturbationDirection;
        
        end                     % Power decreased: reverse the next duty step.
        
        mpptDutyRequest = min(max(mpptDutyRequest + perturbationDirection*mpptDutyStep, 0), maximumBoostDuty);
    
    end                         % First observation establishes the reference.
    
    previousPvPower = pvPower; previousPowerIsValid = true; mpptSampleCount = 0;

end

boostDutyCycle = min(mpptDutyRequest, batteryDutyCeiling);

%% UPDATE BATTERY PI MEMORY -- avoid accumulating beyond the duty limits.

if (currentLimitedDuty > 0 && currentLimitedDuty < maximumBoostDuty) || (currentLimitedDuty == 0 && chargingCurrentError > 0) || (currentLimitedDuty == maximumBoostDuty && chargingCurrentError < 0)
    
    currentLimitIntegral = currentLimitIntegral + chargeCurrentIntegralGain*controlPeriod*chargingCurrentError;

end

if (voltageLimitedDuty > 0 && voltageLimitedDuty < maximumBoostDuty) || (voltageLimitedDuty == 0 && chargingVoltageError > 0) || (voltageLimitedDuty == maximumBoostDuty && chargingVoltageError < 0)
    
    voltageLimitIntegral = voltageLimitIntegral + chargeVoltageIntegralGain*controlPeriod*chargingVoltageError;

end

end