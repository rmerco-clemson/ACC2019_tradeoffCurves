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

% this script will create the plot in "Analysis of Maximum
% Allowable Transmission Interval and Maximum Allowable Delay for Dynamic
% Output Feedback Networked Control Systems in Presence of Constraints on
% Control Performance" submitted at the conference ACC 2019.

figure(1)
hold on
load results_lambdas_clean
load results_gammas_clean
plot(TsMaxG1,TdMaxVectG1,'linewidth',2)
plot(TsMaxG2,TdMaxVectG2,'linewidth',2)
plot(TsMaxG3,TdMaxVectG3,'linewidth',2)
plot(TsMaxL2,TdMaxVectL2,'--','linewidth',2)
plot(TsMaxL3,TdMaxVectL3,'--','linewidth',2)
plot(TsMaxL4,TdMaxVectL4,'--','linewidth',2)
plot(TsMaxG4,TdMaxVectG4,'linewidth',2)
plot(TsMaxG5,TdMaxVectG5,'linewidth',2)
hold off
grid on
xlabel('T_{mati} [s]')
ylabel('T_{mad} [s]')
title('Tradeoff curves between T_{mati}, T_{mad} for varying \lambda_t and \gamma')
legend({'\lambda_t = 0; \gamma = 2.5','\lambda_t = 0; \gamma = 3', ...
    '\lambda_t = 0; \gamma = 5','\lambda_t = 0.5 \lambda_{min}(A11); \gamma = 5','\lambda_t = 0.8 \lambda_{min}(A11); \gamma = 5', ...
    '\lambda_t = 0.89 \lambda_{min}(A11); \gamma = 5','\lambda_t = 0; \gamma = 10','\lambda_t = 0; \gamma = 200'},'Location','north', ...
    'NumColumns',2)
ylim([0 0.045])
xlim([0 0.045])