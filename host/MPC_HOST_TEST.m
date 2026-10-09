%% MPC_HOST_TEST.m
% Host side of the automated MPC test protocol. Pairs with the firmware in
% src/test_harness_mpc.cpp (build/upload it with:  pio run -e test_mpc -t upload).
%
% This is MPC_HOST.m with three changes, all of them required for a test
% run rather than a demo:
%
%   1) IT LOGS. MATLAB holds the COM port exclusively, so PlatformIO's
%      serial monitor cannot attach and log2file cannot run. If this script
%      does not write the file, the run produces no data at all. Every line
%      received is appended to logs/mpc-test-<timestamp>.csv verbatim, so
%      that file is the exact equivalent of the PID harness's log and loads
%      with the same parser.
%
%   2) IT WORKS IN DEVIATION VARIABLES. The plant is really
%      u -> (T - Tambient), not u -> T. A model in absolute temperature
%      predicts 0 degC at zero duty, so the controller sees a ~25 degC
%      phantom error and saturates permanently. The firmware now sends
%      ambient as a third field (",A:25.10") and this script controls
%      (T - A) against (SP - A). This is the single most important
%      difference from MPC_HOST.m -- without it the MPC does not work.
%
%   3) THE PLANT MODEL IS IDENTIFIED, NOT A PLACEHOLDER. K and tau below
%      were fitted from the logged open-loop step response in
%      logs/device-monitor-260812-064112.log (phase T1_HEAT: 2378 samples
%      at a constant u=255).
%
% The firmware stays silent during its 2-minute baseline, so nothing will
% print for the first ~2 minutes. That is expected: asking the MPC for
% commands while the relays are parked would corrupt its state estimate.

clear; close all; clc;

%% ------------------------------------------------------------------
%  1) IDENTIFIED PLANT MODEL
%  --------------------------------------------------------------------
% Fitted from the measured step response:
%   step start  t=123 s, T0 = 26.30 degC, ambient 25.51 degC
%   plateau     Tinf   = 28.65 degC  = ambient + 3.14 degC at u = 255
%   63.2% point 27.79 degC, first reached at t = 576 s
%
%   tau = 576 - 123                = 454 s
%   K   = (28.65 - 25.51) / 255    = 0.01232 degC per duty unit
%
% Note how far these are from the placeholders in MPC_HOST.m (tau 90 s,
% K 0.03): tau is 5x longer and K is 2.4x smaller. Re-fit these if you
% change the heater, the enclosure, or the insulation.
K_identified   = 0.01232;   % degC per duty unit, ABOVE AMBIENT
tau_identified = 454;       % seconds

Ts = 1;  % MUST match Ts in test_harness_mpc.cpp

sys_c = tf(K_identified, [tau_identified, 1]);
sys_d = c2d(sys_c, Ts);

%% ------------------------------------------------------------------
%  2) MPC CONTROLLER
%  --------------------------------------------------------------------
sys_mpc = setmpcsignals(sys_d, 'MV', 1);

% Horizons scaled to the identified tau, not to the paper's room model.
% p*Ts should cover a useful fraction of tau; with tau = 454 s and Ts = 1 s
% a 15-step horizon sees only 3% of the response and the MPC behaves like a
% short-sighted P controller. 300 steps ~ 0.66*tau is a reasonable start.
% If mpcmove is too slow to keep up with Ts = 1 s, reduce p before anything
% else -- the loop must close every second.
p = 300;   % prediction horizon [steps]
m = 3;     % control horizon [steps]

mpcobj = mpc(sys_mpc, Ts, p, m);

mpcobj.MV.Min = -255;
mpcobj.MV.Max =  255;
mpcobj.MV.RateMin = -60;
mpcobj.MV.RateMax =  60;

mpcobj.Weights.OutputVariables          = 1;
mpcobj.Weights.ManipulatedVariables     = 0;
mpcobj.Weights.ManipulatedVariablesRate = 0.1;

xmpc = mpcstate(mpcobj);

%% ------------------------------------------------------------------
%  3) SERIAL CONNECTION
%  --------------------------------------------------------------------
port = "COM3";
baud = 9600;

s = serialport(port, baud);
configureTerminator(s, "LF");
flush(s);
pause(2);   % Arduino auto-resets when the port opens; let it boot

%% ------------------------------------------------------------------
%  4) LOG FILE  (this is the run's only data record)
%  --------------------------------------------------------------------
% The log path is PINNED, not derived from mfilename. Deriving it broke the
% previous run: mfilename('fullpath') returns '' when a script is pasted
% into the Command Window or run in ways that do not bind a file, so the
% path silently resolved somewhere else (or nowhere) and a 57-minute
% session produced no data file at all. An absolute path cannot do that.
logDir = 'C:\Users\ramon.DESKTOP-2KJCUOT\Documents\PlatformIO\Projects\final project lbycpc3\logs';
if ~exist(logDir, 'dir'); mkdir(logDir); end
logPath = fullfile(logDir, sprintf('mpc-test-%s.csv', datestr(now, 'yymmdd-HHMMSS')));
fid = fopen(logPath, 'w');
if fid < 0
    error('Could not open log file %s -- fix this before running, or the test produces no data.', logPath);
end
% Prove the file exists and is writable NOW, rather than discovering at the
% end of a two-hour run that it never was.
fprintf(fid, '# log opened %s\n', datestr(now));
fprintf('LOGGING TO: %s\n', logPath);
if exist(logPath, 'file') ~= 2
    error('Log file %s was not created. Stop and fix this before running.', logPath);
end
fprintf('Connected to %s. The board is silent for its first ~2 min (baseline).\n', port);
fprintf('Press the STOP button on the rig, or Ctrl+C here, to abort.\n\n');

%% ------------------------------------------------------------------
%  5) MAIN LOOP
%  --------------------------------------------------------------------
logT = []; logSP = []; logU = []; logTime = [];
startTime = tic;
sawSummary = false;

% fclose('all') rather than fclose(fid): the flush below reopens the file
% and changes fid, so a closure capturing the original handle would close
% the wrong one.
cleanupObj = onCleanup(@() fclose('all'));
linesSinceFlush = 0;

try
    while true
        if s.NumBytesAvailable > 0
            line = strtrim(readline(s));

            % Everything the board says goes to the log verbatim, including
            % the '#' commentary and the final RESULT/DUTY block. Do this
            % before any parsing so nothing can be lost to a parse error.
            fprintf(fid, '%s\n', line);

            % MATLAB has no fflush for files, and buffered writes are lost
            % if the process is killed rather than interrupted. Closing and
            % reopening in append mode every ~30 lines (about 15 s of run)
            % forces the data to disk, so the worst case loses seconds
            % instead of hours.
            linesSinceFlush = linesSinceFlush + 1;
            if linesSinceFlush >= 30
                fclose(fid);
                fid = fopen(logPath, 'a');
                linesSinceFlush = 0;
            end

            % Surface the board's commentary AND its results in the console.
            %
            % Printing only '#' lines was a mistake: the RESULT and DUTY
            % rows -- the actual deliverable -- carry no '#', so the last
            % run's summary looked empty on screen even though the board
            % had sent it. Anything that is not a numeric CSV sample row is
            % worth showing.
            isSample = ~isempty(regexp(line, '^\d+\.\d+,', 'once'));
            if ~isempty(line) && ~isSample
                fprintf('%s\n', line);
                if contains(line, 'RESULTS')
                    sawSummary = true;
                end
                if contains(line, 'OPEN THE ENCLOSURE')
                    beep; fprintf(2, '>>> OPEN THE LID NOW, HOLD 5 s <<<\n');
                end
                if contains(line, 'CLOSE AND RESEAL')
                    beep; fprintf(2, '>>> CLOSE AND RESEAL NOW <<<\n');
                end
            end

            % Control line: "T:26.30,SP:27.40,A:25.10"
            tok = regexp(line, 'T:([-\d.]+),SP:([-\d.]+),A:([-\d.]+)', 'tokens');
            if ~isempty(tok)
                T_meas  = str2double(tok{1}{1});
                SP_meas = str2double(tok{1}{2});
                A_meas  = str2double(tok{1}{3});

                if ~isnan(T_meas) && ~isnan(SP_meas) && ~isnan(A_meas)
                    % Deviation variables -- see note 2 in the header.
                    y_dev   = T_meas  - A_meas;
                    ref_dev = SP_meas - A_meas;

                    u = mpcmove(mpcobj, xmpc, y_dev, ref_dev);

                    if isnan(u); u = 0; end
                    u = max(-255, min(255, u));

                    write(s, sprintf("U:%.1f\n", u), "string");

                    logT(end+1)    = T_meas;      %#ok<SAGROW>
                    logSP(end+1)   = SP_meas;     %#ok<SAGROW>
                    logU(end+1)    = u;           %#ok<SAGROW>
                    logTime(end+1) = toc(startTime); %#ok<SAGROW>

                    fprintf('t=%6.1fs  T=%5.2f  SP=%5.2f  amb=%5.2f  u=%7.1f\n', ...
                        logTime(end), T_meas, SP_meas, A_meas, u);
                end
            end
        else
            pause(0.02);
        end
    end
catch ME
    if ~strcmp(ME.identifier, 'MATLAB:custom:userinterrupt')
        fprintf(2, 'Loop stopped: %s\n', ME.message);
    end
end

clear s;

if sawSummary
    fprintf('\nRun completed. Summary block is at the end of:\n  %s\n', logPath);
else
    fprintf(2, '\nStopped before the summary block was received.\n');
    fprintf(2, 'Partial data is in %s\n', logPath);
end

%% ------------------------------------------------------------------
%  6) PLOT
%  --------------------------------------------------------------------
if ~isempty(logTime)
    figure('Name','Live MPC Test Run');
    subplot(2,1,1);
    plot(logTime, logT, 'b', logTime, logSP, 'k--', 'LineWidth', 1.3);
    legend('Measured Temp','Setpoint'); ylabel('degC'); grid on;
    title('Hardware MPC: Temperature Tracking');

    subplot(2,1,2);
    stairs(logTime, logU, 'r', 'LineWidth', 1.2);
    ylabel('Duty command'); xlabel('Time [s]'); grid on;
    title('MPC Control Effort');
end
