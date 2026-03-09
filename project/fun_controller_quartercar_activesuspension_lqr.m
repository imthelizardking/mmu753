function [A_cl, K] = fun_controller_quartercar_activesuspension_lqr(A,B,C,D,q1,q2,q3,q4,R)
% Define Weighting Matrices
% Adjust these values to tune performance:
% Q should be 4x4 (for 4 states). 
% We focus on body travel and wheel travel.

Q = diag([q1, q2, q3, q4]);

% 3. Compute the Optimal Gain K
[K, S, P] = lqr(A, B(:,2), Q, R);

% 4. Create the Closed-Loop System
% The new state matrix is (A - B_ctrl*K)
A_cl = A - B(:,2)*K;

% The closed-loop system still reacts to road disturbance (B(:,1))
% but uses the feedback law for the actuator.

fprintf('LQR Gain Matrix K computed successfully.\n');
disp(K);
end
