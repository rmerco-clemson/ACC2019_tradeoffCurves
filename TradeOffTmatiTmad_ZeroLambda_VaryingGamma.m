% Created by Roberto Merco, July 2018, 2018

% Copyright (c) 2018, Roberto Merco
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%
%    * Redistributions of source code must retain the above copyright
%      notice, this list of conditions and the following disclaimer.
%    * Redistributions in binary form must reproduce the above copyright
%      notice, this list of conditions and the following disclaimer in the
%      documentation and/or other materials provided with the distribution.
%    * Neither the name of the Willow Garage, Inc. nor the names of its
%      contributors may be used to endorse or promote products derived from
%       this software without specific prior written permission.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.

% this file was created to generate plots on the paper "Analysis of Maximum
% Allowable Transmission Interval and Maximum Allowable Delay for Dynamic
% Output Feedback Networked Control Systems in Presence of Constraints on
% Control Performance" submitted at the conference ACC 2019. Running this
% code will generate a plot which analyze the trade-off curves between MATI
% and MAD for different values of L2 gain gamma and constant zero decay
% rate.

%% 
clear all
close all
clc

format long
 
%% model known variables definition
% xdot-x
A11 = [1.38 -0.2077 6.7150 -5.6760 0 0;
       -0.5814 -15.6480 0 0.6750 -11.3580 0;
       -14.6630 2.001 -22.3840 21.6230 -2.2720 -25.1680;
       0.0480 2.001 1.3430 -2.1040 -2.2720 0;
       0 1 0 0 0 0;
       1 0 1 -1 0 0];

% xdot-e
A12 = [0 0;
       0 -11.3580;
       -15.73 -2.2720;
       0 -2.2720;
       0 1;
       1 0];

% xdot-omega
A13 = [10 0 10 0 0 0;
       0 5 0 5 0 0]';

% edot-x
A21= [13.3310 0.2077 17.0120 -18.0510 0 25.1680;
      0.5814 15.648 0 -0.6750 11.3580 0];
 
%edot-e
A22 = [15.73 0; 
       0 11.3580];

% edot-omega
A23 = [0 0; 0 0];

% performance output
Co = [1 0 1 -1 0 0;
     0 1 0 0 0 0]; 
Do = [0 0; 0 0];

%% LMI known variables definition
lambda = 0;
gammaVect = [2.5 3 5 10 200].^2;

%% LMI unknown variables definition
nx = size(A11,1);
P1 = sdpvar(nx,nx,'symmetric');
neta = size(A21,1);
P20 = sdpvar(neta,neta,'symmetric');
P30 = sdpvar(neta,neta,'symmetric');
P21 = sdpvar(neta,neta,'symmetric');
P31 = sdpvar(neta,neta,'symmetric');

%% LMI Loop

%loop on gamma
for gi = 1:length(gammaVect)
    
    gamma = gammaVect(gi); 
    if (gi == 1)
        TsMax = [0.0035:0.0005:0.009]; % gamma1
        TsMaxG1 = TsMax; %for future elaboration        
        dstart = 50; %for gamma 1-2
    elseif (gi == 2)
        TsMax = [0.011:0.0005:0.02,0.0201:0.0001:0.0204]; % gamma2 
        TsMaxG2 = TsMax; %for future elaboration
        dstart = 50; %for gamma 1-2
    elseif (gi == 3)
        TsMax = [0.0215:0.0005:0.031,0.0315,0.03175:0.00001:0.0318]; % gamma3
        TsMaxG3 = TsMax; %for future elaboration
        dstart = 70; %for gamma 3,4,5
    elseif (gi == 4)
        TsMax = [0.0255:0.0005:0.037,0.0375,0.0377:0.00001:0.03772]; % gamma4
        TsMaxG4 = TsMax; %for future elaboration
        dstart = 70; %for gamma 3,4,5
    elseif (gi == 5)
        TsMax = [0.0285:0.0005:0.042,0.04201:0.00001:0.04206]; % gamma5
        TsMaxG5 = TsMax; %for future elaboration
        dstart = 70; %for gamma 3,4,5
    end
    
    TdMaxVect = -ones(size(TsMax));
    dMax = -ones(size(TsMax));
                                
    for i=1:length(TsMax)
        disp('Ts:')
        Ts = TsMax(i)

        % max Td per Ts 
        TdMax = 0;
        % variable counter to quit loop delta
        countFalse = 0;

        if (i>1)             
            dstart = dMax(i-1); 
        end

        % run in the values of delta
        for d= dstart:0.05:100 %1:0.1:10

            % var to quit loop when with d no Td is found
            exitVar = 0;

            % if the previous iteraction you had zero delay quit the cycle
            if (i>1)             
                if (TdMaxVect(i-1)<=0)
                    break; 
                end
            end

            % run through the values of allowable delays
            for Td = TdMax:0.0001:Ts  %0.001

                %M_Tl
                M_01 = buildM(P1,P21,P31,A11,A12,A13,A21,A22,A23,Co,Do,lambda,gamma,d,0);
                M_Td1 = buildM(P1,P21,P31,A11,A12,A13,A21,A22,A23,Co,Do,lambda,gamma,d,Td);
                M_Td0 = buildM(P1,P20,P30,A11,A12,A13,A21,A22,A23,Co,Do,lambda,gamma,d,Td);
                M_Ts0 = buildM(P1,P20,P30,A11,A12,A13,A21,A22,A23,Co,Do,lambda,gamma,d,Ts);

                % Constraints
                Constraints = [P1>=0, P20>=0, P30>=0, P21>=0, P31>=0, P20-P21<=0, P31-exp(-d*Ts)*P20<=0, M_01<=0, M_Td1<=0, M_Td0<=0, M_Ts0<=0];

                % optimization problem
                % Solve the problem
                Objective = [];
                options = sdpsettings('solver','sdpt3');
                diagnostics = optimize(Constraints,Objective,options);

                % Analyze error flags
                if diagnostics.problem == 0
                 disp('Solver thinks it is feasible: i=')             
                 if (Td > TdMaxVect(i))
                    TdMaxVect(i) = Td;
                    TdMax = Td;
                    dMax(i) = d;
                    exitVar = 1; % found a better time, don't stop the loop
                    % reset variable to quit loop delta
                    countFalse = 0;
                 end
                 Ts
                 Td
                 d
                 TdMax
                 TdMaxVect
                else
                 disp('Something else happened')
                 Ts
                 Td
                 d
                 TdMax
                 TdMaxVect
                 % increment variable to quit loop delta
                 countFalse = countFalse+1;
                 % exit loop if delta is not feaseble
                 break;
                end

                % if your delay is as much as your sample time, quit
                if (TdMaxVect(i)==Ts) 
                    break; 
                end
            end

            % if your delay is as much as your sample time, quit
            if (TdMaxVect(i)==Ts) 
                break; 
            end

            % if no delta satisfy Td and you have already a Td, quit the loop
            if (exitVar == 0 && TdMaxVect(i)~=-1)
                % to avoid some delta which not satisfy in small regions
                if (countFalse>5)
                    break;
                end
            end                 
        end    
    end
    
    if (gi == 1)
        TdMaxVectG1 = TdMaxVect; %for future elaboration        
    elseif (gi == 2)
        TdMaxVectG2 = TdMaxVect; %for future elaboration 
    elseif (gi == 3)
        TdMaxVectG3 = TdMaxVect; %for future elaboration
    elseif (gi == 4)
        TdMaxVectG4 = TdMaxVect; %for future elaboration
    elseif (gi == 5)
        TdMaxVectG5 = TdMaxVect; %for future elaboration
    end
end


%% plot
figure(1)
hold on
% load results_gammas_clean
plot(TsMaxG1,TdMaxVectG1,'linewidth',2)
plot(TsMaxG2,TdMaxVectG2,'linewidth',2)
plot(TsMaxG3,TdMaxVectG3,'linewidth',2)
plot(TsMaxG4,TdMaxVectG4,'linewidth',2)
plot(TsMaxG5,TdMaxVectG5,'linewidth',2)
hold off
grid on
xlabel('T_{mati} [s]')
ylabel('T_{mad} [s]')
title('Tradeoff curves between T_{mati}, T_{mad} and L_2 performance for \lambda_t = 0')
% ylim([0 0.02])
legend('\gamma = 2.5','\gamma = 3','\gamma = 5','\gamma = 10','\gamma = 200')