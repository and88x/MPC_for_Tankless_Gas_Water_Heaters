%**************************************************************************
% Autor: Fernando Garcia
% Email: fernando.garcia88x@gmail.com
% Date: 26/04/2020
%**************************************************************************
clear all; close all; clc
%% Simulation Parameters
t  = 120;                                   % Simulation time
Ts = 0.25;                                  % Sampling time
N  = floor(t/Ts);                           % # of samples per sim
Np = 50;                                    % Prediction horizon
r  = 50;                                    % Fixed reference
xmeasure = 15;                              % Initial conditions

%% System Parameters
T_in = 15;                  Power_max = 58300;
ro   = 1000;                Power_min = 2600;
Cpw  = 4186;                q_max     = 50/60000;
Cpm  = 385;                 T_max     = 100;
ri   = 8e-3;                T_min     = 0;
Li   = 2.2; 
Mw   = 2; %[Kg]
Mm   = 2; %[Kg]

% Delays and increments
DeltaTin    = 0.75;     % [?C] Temperature incremenent at the heat cell inlet
DeltaTout   = 0.75;     % [?C] Temperature incremenent at the heat cell outlet
Power_delay = 2.5;      % [s] Thermal power delay
RPD         = ceil(Power_delay/Ts);      % Rounded Power Delay for Ts
deltaT_ss   = 48.43 - T_in;
q_ss        = 10/60000;                  % Flow rate for initial conditions
Flow_delay  = pi*ri^2*Li/(q_ss+2e-7);    % Delay for initial conditions
nn          = 13; 

%% Initialize the system
Power_input     = 1;                     % Initial condition for the simulink model
flow_rate_input = 1/6000;                % Initial condition for the simulink model
x0              = T_in;                  % Initial condition

load_system('TGWH3.slx');
myOperPoint = Simulink.BlockDiagram.getInitialState('TGWH3');
u_new       = zeros(1,Np);               % Initial condition for u
aux         = ceil(N/3);                 % Only for flow_signal
flow_signal = 1/60000*[10*ones(1,aux) 3*ones(1,aux) 10*ones(1,aux)];
uc          = zeros(1,N);                % For plot
xc          = zeros(1,N);                % For plot
tc          = zeros(1,N);                % For plot

RFD      = ceil(Flow_delay/Ts);   
prev_RFD = RFD;
%% Model Predictive Control loop
tic
for k = 1 : N - 1 
    Flow_delay = pi*ri^2*Li/(flow_signal(k)+2e-7); 
    RFD        = ceil(Flow_delay/Ts);       % Rounded flow delay for Ts
    
    % Smoother delay
%     if prev_RFD < RFD
%         RFD = prev_RFD + 1;
%     end
%     prev_RFD = RFD;
    % -----------------------------------------------------
        % First step of the MPC algorithm            
    t0 = (k - 1)*Ts;                        % Current time
    u0 = [u_new(2:end) u_new(end)];         % ShiftHorizon
    x0 = xmeasure;                          % Real measure
    flow_rate_input = flow_signal(k);       % For simulink model
    delayed_in      = u0(1:RPD+RFD);        % For simulate the time delays
    prediction      = u0(RPD+RFD+1:Np);     % Signals to be predicted
    
    % -----------------------------------------------------
        % Second step of the MPC algorithm     
    [u, V] = solveOCP(Ts, x0, delayed_in, prediction, flow_rate_input, Np, r, t0);
    
    % -----------------------------------------------------
        % Third step of the MPC algorithm
    u_new                = [u_new(2:end) u_new(end)]; 
    u_new(RPD+RFD+1:end) = u;

    uc(k)       = u(1);                      % For plot                                                                                                 
    Power_input = u(1);                      % For simulink model
    simOut = sim('TGWH3',...
                    'StopTime', num2str(k*Ts),...
                    'SaveFinalState','on',...
                    'FinalStateName','myOperPoint',...
                    'SaveOperatingPoint','on',...
                    'LoadInitialState','on',...
                    'InitialState','myOperPoint');
    myOperPoint = simOut.myOperPoint;        % Last state in the simulink model
    xmeasure    = simOut.yout{1}.Values.Data(end);
    xc(k)       = xmeasure;
    tc(k)       = t0;
    % ----------------------------------------------------- 
            % Plot the system response
    subplot(2,2,[1,2])
    plot(tc(1:k),xc(1:k),'b'); title('Temperature');grid on;
    subplot(2,2,3)
    plot(tc(1:k),uc(1:k)); title('Power');grid on;
    subplot(2,2,4)
    plot(tc(1:k),flow_signal(1:k)); title('Flowrate');grid on;
%     pause(0.0001)
end        
toc