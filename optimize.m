% optimize

function fmin = optimize()
    
    global param
    
    % Maximum number of evaluations before stopping the algorithm
    opt.maxeval = 100;
    
    % Relative stopping tolerance (e.g., xtol_rel = .01 means the
    % optimization will stop when a step changes all parameters by less
    % than 1%)
    opt.xtol_rel = 0.00001;
    
    % Choose optimization algorithm
    opt.algorithm = NLOPT_GN_CRS2_LM;
    
    % Set optimization bounds
    tauMin1 = 0;
    tauMax1 = 0.5;
    tauMin2 = 0;
    tauMax2 = 0.5;
    thStartMin1 = 0;
    thStartMax1 = pi;
    thStartMin2 = 0;
    thStartMax2 = pi;
    timeTotalMin = 1;
    timeTotalMax = 10;
    
    % Parameters: [tau1, tau2, initial theta1, initial theta2, time duration]
    opt.lower_bounds = [tauMin1,tauMin2,thStartMin1,thStartMin2,timeTotalMin];
    opt.upper_bounds = [tauMax1,tauMax2,thStartMax1,thStartMax2,timeTotalMax];
    
    % Initial guess for first optimization evaluation
    init_guess = [(tauMax1-tauMin1)/2, (tauMax2-tauMin2)/2,...
        (thStartMax1-thStartMin1)/2, (thStartMax2-thStartMin2)/2,...
        (timeTotalMax-timeTotalMin)/2];
    
    % Set objective function handle
    opt.min_objective = @objFunc;
    
    % Run NLopt algorithm.
    % @xopt: parameters associated with minimum objective function value
    % @fmin: minimum objective function value (RMS error)
    [xopt, fmin, ~] = nlopt_optimize(opt, init_guess);
    
    % Assign optimal parameters to param structure
    param.tau1 = xopt(1);
    param.tau2 = xopt(2);
    param.th1_0 = xopt(3);
    param.th2_0 = xopt(4);
    param.time_total = xopt(5);