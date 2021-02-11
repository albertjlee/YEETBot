% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(lambda1*quad_form(idMat1*(b - G*n*sig), eye(3)) + lambda2*quad_form(idMat2*(b - G*n*sig), eye(3)) + lambda3*quad_form(n*sig, eye(12)))
%   subject to
%     0 <= sig
%
% with variables
%      sig  16 x 1
%
% and parameters
%        G   6 x 12
%        b   6 x 1
%   idMat1   3 x 6
%   idMat2   3 x 6
%  lambda1   1 x 1    positive
%  lambda2   1 x 1    positive
%  lambda3   1 x 1    positive
%        n  12 x 16
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.G, ..., params.n, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2020-11-11 20:11:32 -0500.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
