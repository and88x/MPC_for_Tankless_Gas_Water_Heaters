%% ===================== + Optimization functions + =======================
function [u, V] = solveOCP_incomplete(Ts, x0, du, u0, q_ss, Np, r, t0)    
% Solves the Optimization Constrained Problem
% u are the predicted responses
% V are the cost function value for u
    aux = size(u0,2);
    A = [-1*eye(aux); eye(aux)];
    b = [zeros(1,aux) ones(1,aux)];
    % Constrains
    Aeq = []; beq = []; lb = []; ub = [];  
    warning off all
    options = optimset('Display','off',...
                'TolFun', 1e-6,...
                'MaxIter', 2000,...
                'Algorithm', 'interior-point',...
                'AlwaysHonorConstraints', 'bounds',...
                'FinDiffType', 'forward',...
                'HessFcn', [],...
                'Hessian', 'bfgs',...
                'HessMult', [],...
                'InitBarrierParam', 0.1,...
                'InitTrustRegionRadius', sqrt(size(u0,1)*size(u0,2)),...
                'MaxProjCGIter', 2*size(u0,1)*size(u0,2),...
                'ObjectiveLimit', -1e20,...
                'ScaleProblem', 'obj-and-constr',...
                'SubproblemAlgorithm', 'cg',...
                'TolProjCG', 1e-2,...
                'TolProjCGAbs', 1e-10);
    % Optimizer
    [u, V] = fmincon(@(u) costfunction(Ts,x0,du,u,q_ss,Np,r,t0),u0,A,b,... 
                            Aeq,beq,lb,ub,@(u) nonlcon(u),options);  
end
% =========================================================================
% 
%