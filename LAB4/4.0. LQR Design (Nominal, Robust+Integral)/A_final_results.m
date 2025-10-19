%% (S7) - (S8): LQR Design (Nominal Tracking Case)

run("Plant_balrob_params.m");
run("Plant_balrob_calculations.m");

% System linearized around theta = 0

% M, G, Fv have to be in the workspace at this point!

M11 = 2*wheel.Iyy + 2*gbox.N^2*mot.rot.Iyy + (body.m + 2*wheel.m + 2*mot.rot.m)*wheel.r^2;
M12 = 2*gbox.N*(1-gbox.N)*mot.rot.Iyy + (body.m*body.zb + 2*mot.rot.m*mot.rot.zb)*wheel.r;
M21 = M12;
M22 = body.Iyy + 2*(1-gbox.N)^2 *mot.rot.Iyy + body.m*body.zb^2 + 2*mot.rot.m*mot.rot.zb^2;

M = [M11, M12 ; M21, M22];

G11 = 0;
G12 = G11;
G21 = G11;
G22 = -(body.m*body.zb + 2*mot.rot.m*mot.rot.zb)*g;

G = [G11 G12; G21 G22];

Fv11 = 2*(gbox.B + wheel.B);
Fv12 = -2*gbox.B;
Fv21 = Fv12;
Fv22 = 2*gbox.B;

Fv = [Fv11 Fv12; Fv21 Fv22];

Fv_prime = Fv + (2*gbox.N^2*mot.Kt*mot.Ke)/mot.R*[1 -1; -1 1];

%Continuous system matrices

A = [zeros(2,2), eye(2); -inv(M)*G, -inv(M)*Fv_prime];

B = (2*gbox.N*mot.Kt)/mot.R * [zeros(2,2); inv(M)]*[1;-1];

%Exact System Discretization

sysC = ss(A,B,zeros(1,4),0);

sysD = c2d(sysC,Ts,'zoh');

Phi = sysD.A;
Gamma = sysD.B;

H = [1 0 0 0];

%Feedforward action Nx, Nu

Mat = [Phi-eye(4) Gamma; H 0];
Sol = inv(Mat)\[zeros(4,1); 1];

Nx = Sol(1:4,:);
Nu = Sol(5,:);

%% LQR Setup via Bryson's Rule:

gamma_bar = pi/36;
theta_bar = pi/360;
u_bar = 1;

Q = diag([1/gamma_bar^2, 1/theta_bar^2, 0, 0]);

r = 1/u_bar^2;

%rho chosen between (500-5000)

rho_vals = [500, 5000];  % Candidate values for rho

K_vals = cell(1, length(rho_vals));  % Preallocate cell array

for i = 1:length(rho_vals)

    % Compute LQR gain for each rho

    [K_fb, ~, ~] = dlqr(Phi, Gamma, Q, rho_vals(i) * r);
    
    % Store in cell array

    K_vals{i} = K_fb;
end

% placed here so that sim model can run
gamma_ref = 0; 
K_i = 0;
open('Model_Controlled_sys.slx');                 % make sure the model is open

% Turn off Integral Action for Robust Tracking

set_param('Model_Controlled_sys/Balance and position Controller/Integral Action','Value','2');

function T = run_nominal_test(x0, gamma_ref, taskTag, taskDesc, plotColor)
%RUN_NOMINAL_TEST  Batch-test the LAB-4 LQR design.
%
%   T = RUN_NOMINAL_TEST(x0, gamma_ref, taskTag, taskDesc)
%
%   x0        : 4×1 initial state   [γ  ϑ  γ̇  ϑ̇]ᵀ
%   gamma_ref : scalar reference for γ
%   taskTag   : short tag, e.g. 'T1' or 'Tilt5deg'
%   taskDesc  : long description, e.g. 'Non-zero IC, no disturbance'
%
%   Outputs
%   --------
%   • Table T of step-response metrics
%   • CSV  <taskTag>_metrics.csv
%   • PNG  <taskTag>_step_metrics.png
%
%   Requires rho_vals, K_vals, and the model Controlled_sys.slx
%   to be available in the base workspace.

% ---------- sanity checks ------------------------------------------------
if nargin < 4, taskDesc = ''; end
assert(evalin('base','exist(''rho_vals'',''var'')'), ...
    'rho_vals missing in base workspace');
assert(evalin('base','exist(''K_vals'',''var'')'), ...
    'K_vals missing in base workspace');

rho_vals = evalin('base','rho_vals');
K_vals   = evalin('base','K_vals');

% ---------- push IC & reference ------------------------------------------
assignin('base','x0',        x0);
assignin('base','gamma_ref', gamma_ref);

open_system('Model_Controlled_sys','load');
set_param('Model_Controlled_sys','SimulationCommand','update');

% ---------- simulate -----------------------------------------------------
n = numel(rho_vals);
riseT = zeros(n,1); settleT = riseT; overshootPct = riseT;

for k = 1:n
    assignin('base','K_fb',K_vals{k});
    simOut = sim('Model_Controlled_sys','StopTime','30','SaveOutput','on');
    g_ts   = simOut.gamma_out;
    S      = stepinfo(g_ts.Data, g_ts.Time, ...
                      'SettlingTimeThreshold',0.02);
    riseT(k)        = S.RiseTime;
    settleT(k)      = S.SettlingTime;
    overshootPct(k) = S.Overshoot;
end

% ---------- table + CSV --------------------------------------------------
T = table(rho_vals(:), riseT, settleT, overshootPct, ...
          'VariableNames',{'rho','RiseTime','SettlingTime','Overshoot'});

disp(['=== ' taskTag ' – ' taskDesc ' ==='])
disp(T)

safeTag = regexprep(taskTag,'\W','_');   % filenames: keep only [A–Z a–z 0–9 _]
csvName = [safeTag '_metrics.csv'];
pngName = [safeTag '_step_metrics.png'];

writetable(T, csvName);

% ---------- plots --------------------------------------------------------
if nargin < 5 || isempty(plotColor), plotColor = 'b'; end   % default blue

figure('Name',[taskTag ' – ' taskDesc], 'Position',[100 100 900 700]);

x    = 2:2:2*n;                         % spaced axis (centred points)
xlab = '\rho = ' + string(rho_vals);
lw   = 1.5;
ls   = [plotColor '-o'];                % e.g. 'r-o'  'g-o'  'k-o'

subplot(3,1,1); hold on; grid on;
plot(x, riseT, ls, 'LineWidth', lw, 'MarkerFaceColor', 'none');
text(x, riseT+0.02, compose('%.2f', riseT), ...
     'HorizontalAlignment','center','FontSize',9);
xlim([min(x)-1 max(x)+1]); set(gca,'XTick',x,'XTickLabel',xlab);
ylabel('Rise Time (s)');
title([taskTag ' – Rise Time vs \rho']);

subplot(3,1,2); hold on; grid on;
plot(x, settleT, ls, 'LineWidth', lw, 'MarkerFaceColor', 'none');
text(x, settleT+0.02, compose('%.2f', settleT), ...
     'HorizontalAlignment','center','FontSize',9);
xlim([min(x)-1 max(x)+1]); set(gca,'XTick',x,'XTickLabel',xlab);
ylabel('Settling Time (s)');
title([taskTag ' – Settling Time vs \rho']);

subplot(3,1,3); hold on; grid on;
plot(x, overshootPct, ls, 'LineWidth', lw, 'MarkerFaceColor', 'none');
text(x, overshootPct+0.3, compose('%.1f', overshootPct), ...
     'HorizontalAlignment','center','FontSize',9);
xlim([min(x)-1 max(x)+1]); set(gca,'XTick',x,'XTickLabel',xlab);
ylabel('Overshoot (%)'); xlabel('\rho value');
title([taskTag ' – Overshoot vs \rho']);

saveas(gcf, pngName);

end

%% Task 1: Nonzero Initial Conditions, with zero reference input:

set_param('Model_Controlled_sys/Trigger','Value','1');   % Turn off the Disturbance

% 1) update the variable the controller uses
assignin('base','gamma_ref', gamma_ref);

% 2) update the Step block that drives the scope
set_param('Model_Controlled_sys/Step gamma reference [deg]', ...
          'After', num2str(gamma_ref));      % 'After' is the final level


x0 = [0; pi/36; 0; 0];

run_nominal_test(x0, gamma_ref, 'T1_{NT}', '5° tilt, γ_{ref}=0');


%% Task 2: Zero Initial Conditions, with 10cm drive forward input:

set_param('Model_Controlled_sys/Trigger','Value','1');   % Turn off the Disturbance

% value that represents 10 cm forward in wheel angle (deg)
gRefDeg = (0.1 / wheel.r) * rad2deg;   

% 1) update the variable the controller uses
assignin('base','gamma_ref', gRefDeg);

% 2) update the Step block that drives the scope
set_param('Model_Controlled_sys/Step gamma reference [deg]', ...
          'After', num2str(gRefDeg));      % 'After' is the final level

x0 = [0; 0; 0; 0];

run_nominal_test(x0, gRefDeg, 'T2_{NT}: Undisturbed', '0° tilt, γ_{ref}-> drive 10cm', 'g');


%% Task 3: Same as in Task 2, with Voltage Driver Disturbance at t = 10s;

set_param('Model_Controlled_sys/Trigger','Value','2');   % Turn on the Disturbance

run_nominal_test(x0, gRefDeg, 'T3_{NT}: Disturbed', '0° tilt, γ_{ref}-> drive 10cm', 'r');

% NOTE: Under Nominal Tracking conditions, the Controller fails to recover
% after being perturbed by the disturbance, and cannot track the reference
% as needed.

set_param('Model_Controlled_sys/Trigger','Value','1');   % Turn off the Disturbance



%% (S9–S10)  Build Augmented DT Matrices for Robust Tracking

% Activate Integral Branch
set_param('Model_Controlled_sys/Balance and position Controller/Integral Action','Value','1');

q11_vals = [0.1  1];      % ← make sure this is in the base WS too

% Augmented matrices (independent of q11, rho)
Phi_a   = [1  ,  H ;
           zeros(4,1) , Phi];
Gamma_a = [0 ;
           Gamma];

K_aug = cell(numel(q11_vals), numel(rho_vals));

for iq = 1:numel(q11_vals)
    Q_aug = blkdiag(q11_vals(iq), Q);      % 5×5
    for ir = 1:numel(rho_vals)
        rho = rho_vals(ir);
        K_aug{iq,ir} = dlqr(Phi_a, Gamma_a, Q_aug, rho*r);   % 1×5
    end
end

% -- expose sweep data to run_robust_test ----------------------
assignin('base','q11_vals', q11_vals);
assignin('base','rho_vals', rho_vals);   % already there, but harmless
assignin('base','K_aug',     K_aug);     % <-- the one run_robust_test needs


function T = run_robust_test(x0, gamma_ref, taskTag, taskDesc, styleCell, simTime)
%RUN_ROBUST_TEST  Sweep (q11, rho) gains for robust tracking tests.
%
%   T = RUN_ROBUST_TEST(x0, gamma_ref, taskTag, taskDesc)
%   T = RUN_ROBUST_TEST(..., styleCell, simTime)
%
%   Inputs
%   ------
%   x0, gamma_ref  : initial state and reference          (same as before)
%   taskTag        : short tag  (used in filenames, titles)
%   taskDesc       : long description (in figure titles)
%   styleCell      : e.g. {'r-o','g-s','b-d'}  (one per q11)
%   simTime        : stop time in seconds   (default 20)
%
%   Requires rho_vals, q11_vals, K_aug, and Controlled_sys.slx
%   in the base workspace.  K_aug{iq,ir} must match those grids.

% ------------------------------------------------------------------------
if nargin < 6 || isempty(simTime),  simTime  = 20;  end
if nargin < 5 || isempty(styleCell), styleCell = {'r-o','g-s','b-d','k-^'}; end

% --- pull sweep vectors & gain grid -------------------------------------
rho_vals   = evalin('base','rho_vals');
q11_vals   = evalin('base','q11_vals');
K_aug      = evalin('base','K_aug');     % cell array {iq, ir}

nR = numel(rho_vals);
nQ = numel(q11_vals);

assert(numel(styleCell) >= nQ, ...
    'styleCell must have at least as many entries as q11_vals');

% --- push IC & reference -------------------------------------------------
assignin('base','x0',        x0);
assignin('base','gamma_ref', gamma_ref);

open_system('Model_Controlled_sys','load');
set_param('Model_Controlled_sys','SimulationCommand','update');

% --- preallocate metric arrays ------------------------------------------
Rise   = zeros(nQ,nR);
Settle = zeros(nQ,nR);
OverS  = zeros(nQ,nR);

% ---------------- simulation loop ----------------------------------------
for iq = 1:nQ
    for ir = 1:nR
        gainVec = K_aug{iq, ir};      % 1×5  [Ki  K_fb]
        assignin('base','K_i',  gainVec(1));    % scalar to the integrator path
        assignin('base','K_fb', gainVec(2:5));  % 1×4 row to the state-feedback block
        
        simOut = sim('Model_Controlled_sys', ...
                     'StopTime', num2str(simTime), ...
                     'SaveOutput','on');
        
        g_ts = simOut.gamma_out;
        S    = stepinfo(g_ts.Data, g_ts.Time, ...
                        'SettlingTimeThreshold',0.02);
        
        Rise(iq,ir)   = S.RiseTime;
        Settle(iq,ir) = S.SettlingTime;
        OverS(iq,ir)  = S.Overshoot;
    end
end

% make a grid that pairs every q11 with every rho
[rGrid, qGrid] = meshgrid(rho_vals, q11_vals);   % both are nQ × nR

% Building and Saving Tables
for iq = 1:nQ
    idx = (qGrid(:) == q11_vals(iq));
    Ti  = table(qGrid(idx), rGrid(idx), Rise(idx), Settle(idx), OverS(idx), ...
                'VariableNames',{'q11','rho','RiseTime','SettlingTime','Overshoot'});
    disp(['=== ' taskTag ' – ' taskDesc ' –  q_{11} = ' num2str(q11_vals(iq)) ' ==='])
    disp(Ti)

    tag_i = [regexprep(taskTag,'\W','_') '_q11_' strrep(num2str(q11_vals(iq)) ,'.','p')];
    writetable(Ti, [tag_i '_metrics.csv']);
end

% Plotting Step Response:

figure('Name',[taskTag ' – ' taskDesc], 'Position',[100 100 1000 800]);

x    = 2:2:2*nR;                % centred positions 2,4,6,...
xlab = '\rho = ' + string(rho_vals);
xlimPad = [min(x)-1  max(x)+1];

subplot(3,1,1); hold on; grid on;
for iq = 1:nQ
    plot(x, Rise(iq,:), styleCell{iq}, 'LineWidth',1.5, ...
         'DisplayName',"q_{11} = " + q11_vals(iq));
    text(x, Rise(iq,:)+0.02, compose('%.2f',Rise(iq,:)), ...
         'HorizontalAlignment','center','FontSize',9);
end
set(gca,'XTick',x,'XTickLabel',xlab); xlim(xlimPad);
ylabel('Rise Time (s)');
title([taskTag ' - Rise Time vs \rho']);
legend('show','Location','northwest');

subplot(3,1,2); hold on; grid on;
for iq = 1:nQ
    plot(x, Settle(iq,:), styleCell{iq}, 'LineWidth',1.5, ...
         'DisplayName',"q_{11} = " + q11_vals(iq));
    text(x, Settle(iq,:)+0.02, compose('%.2f',Settle(iq,:)), ...
         'HorizontalAlignment','center','FontSize',9);
end
set(gca,'XTick',x,'XTickLabel',xlab); xlim(xlimPad);
ylabel('Settling Time (s)');
title([taskTag ' - Settling Time vs \rho']);
legend('show','Location','northwest');

subplot(3,1,3); hold on; grid on;
for iq = 1:nQ
    plot(x, OverS(iq,:), styleCell{iq}, 'LineWidth',1.5, ...
         'DisplayName',"q_{11} = " + q11_vals(iq));
    text(x, OverS(iq,:)+0.3, compose('%.1f',OverS(iq,:)), ...
         'HorizontalAlignment','center','FontSize',9);
end
set(gca,'XTick',x,'XTickLabel',xlab); xlim(xlimPad);
ylabel('Overshoot (%)'); xlabel('\rho value');
title([taskTag ' - Overshoot vs \rho']);
legend('show','Location','northwest');

saveas(gcf, [regexprep(taskTag,'\W','_') '_step_metrics.png']);

end

%%  Task 4  – Non-zero IC, γref=0  (blue-green palette)

% 1) update the variable the controller uses
assignin('base','gamma_ref', gamma_ref);

% 2) update the Step block that drives the scope
set_param('Model_Controlled_sys/Step gamma reference [deg]', ...
          'After', num2str(gamma_ref));      % 'After' is the final level

run_robust_test([0;pi/36;0;0], gamma_ref, ...
    'T4_{RT}', '5° tilt, γ_{ref}=0', {'b-o','g-s','m-d','k-^'}, 25);

%% Task 5  – Undisturbed 10 cm drive

% 1) update the variable the controller uses
assignin('base','gamma_ref', gRefDeg);

% 2) update the Step block that drives the scope
set_param('Model_Controlled_sys/Step gamma reference [deg]', ...
          'After', num2str(gRefDeg));      % 'After' is the final level

set_param('Model_Controlled_sys/Trigger','Value','1');
run_robust_test([0;0;0;0], gRefDeg, ...
    'T5_{RT}: Undisturbed', 'Undisturbed 10 cm drive', {'r-o','g-s'}, 25);

%% Task 6  – Disturbed 10 cm drive

set_param('Model_Controlled_sys/Trigger','Value','2');
run_robust_test([0;0;0;0], gRefDeg, ...
    'T6_{RT}: Disturbed', 'Disturbed 10 cm drive', {'r-o','g-s'}, 25);


% Disable Disturbance
set_param('Model_Controlled_sys/Trigger','Value','1'); 

% Disable Integral Branch
set_param('Model_Controlled_sys/Balance and position Controller/Integral Action','Value','2');

close_system("Model_Controlled_sys.slx",0);




