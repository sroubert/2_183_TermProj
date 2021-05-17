% optimize

function fmin = optimize()
    
    global param control
    
    % Maximum number of evaluations before stopping the algorithm
    opt.maxeval = 600;
    
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
    timeTotalMin = 0.01;
    timeTotalMax = 5;
    
    % chosen somewhat arbitrarily
    xiMin=-.45;
    yiMin=0;
    xfMin=-.45;
    yfMin=0;
    xiMax=.45;
    yiMax=0.45;
    xfMax=0.45;
    yfMax=0.45;
    
    
    % Parameter bounds
    if control == "torque"
        % Torque control
        % Parameters: [tau1, tau2, initial theta1, initial theta2, time duration]
        opt.lower_bounds = [tauMin1,tauMin2,thStartMin1,thStartMin2,timeTotalMin];
        opt.upper_bounds = [tauMax1,tauMax2,thStartMax1,thStartMax2,timeTotalMax];
    
    elseif control == "hand position"
        % Cartesian position control
        % Parameters: [Initial position (x & y), , time duration]
        opt.lower_bounds = [xiMin, yiMin, xfMin, yfMin, timeTotalMin];

        opt.upper_bounds = [xiMax, yiMax, xfMax, yfMax, timeTotalMax];
    end
        
    % Initial guess for first optimization evaluation
    if control == "torque"
        % Torque control
        init_guess = [(tauMax1-tauMin1)/2, (tauMax2-tauMin2)/2,...
            (thStartMax1-thStartMin1)/2, (thStartMax2-thStartMin2)/2,...
            (timeTotalMax-timeTotalMin)/2];
    
    elseif control == "hand position"
        % Cartesian position control
        init_guess = [(xiMax-xiMin)/2, (yiMax-yiMin)/2, (xfMax-xfMin)/2,...
            (yfMax-yfMin)/2, (timeTotalMax-timeTotalMin)/2];
    end
    
    % Set objective function handle
    opt.min_objective = @objFunc;
    
    % Run NLopt algorithm.
    % @xopt: parameters associated with minimum objective function value
    % @fmin: minimum objective function value (RMS error)
    [xopt, fmin, ~] = nlopt_optimize(opt, init_guess);
    
    % Assign optimal parameters to param structure
    % Torque control
    param.tau1 = xopt(1);
    param.tau2 = xopt(2);
    param.th1_0 = xopt(3);
    param.th2_0 = xopt(4);
    param.time_total = xopt(5);
    
    % Cartesian position control
    param.xi = xopt(1);
    param.yi = xopt(2);
    param.xf = xopt(3);
    param.yf = xopt(4);
    param.time_total = xopt(5);