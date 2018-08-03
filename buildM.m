function M = buildM(P1,P2,P3,A11,A12,A13,A21,A22,A23,Co,Do,lambda,gamma,d,T)

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

% this function will set up the LMI described in the paper "Analysis of Maximum
% Allowable Transmission Interval and Maximum Allowable Delay for Dynamic
% Output Feedback Networked Control Systems in Presence of Constraints on
% Control Performance" submitted at the conference ACC 2019.

% block elements of the matrix
M11 = A11'*P1+P1*A11 + (Co'*Co) + 2*lambda*P1;
M12 = P1*A12+exp(-d*T)*A21'*P2;
M13 = -P1*A12;
M14 = P1*A13+Co'*Do;
M21 = M12';
M22 = exp(-d*T)*(A22'*P2+P2*A22+(2*lambda-d)*P2);
M23 = zeros(size(M22,1),size(M13,2));
M24 = exp(-d*T)*P2*A23; 
M31 = M13';
M32 = M23';
M33 = exp(-d*T)*(2*lambda-d)*P3;
M34 = zeros(size(M32,1),size(M14,2));
M41 = M14';
M42 = M24';
M43 = M34';
M44 = -gamma*eye(size(A13,2))+Do'*Do;

M=[M11 M12 M13 M14;
   M21 M22 M23 M24;
   M31 M32 M33 M34;
   M41 M42 M43 M44];
