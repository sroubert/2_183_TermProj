%Gets torque values according to tau = K*(delta Theta) + B*(Theta_dot)

function tau = torqueFromImpedance(z,ti,p)
    T=p.time_total;
    t=p.t;
    th0=p.thd;

    
    
    if p.dof==3
        th1=z(1);
        th2=z(2);
        th3=z(3);
        
    
        dth1=z(4);
        dth2=z(5);
        dth3=z(6);
        
        th=[th1;th2;th3];
        dth=[dth1;dth2-dth1;dth3-dth2-dth1];%relative velocities
        
        th0_t=[interp1(t,th0(:,1),ti);interp1(t,th0(:,2),ti);interp1(t,th0(:,3),ti)]; %approx th0 values at specified t
    end
    
    if p.dof==2
        th1=z(1);
        th2=z(2);
    
        dth1=z(3);
        dth2=z(4);
        
        th=[th1;th2]; 
        dth=[dth1;dth2-dth1];%relative velocities
    
        th0_t=[interp1(t,th0(:,1),ti);interp1(t,th0(:,2),ti)]; %approx th0 values at specified t
    end
    
    K=p.K;
    B=p.B;   
    
    tau=K*[th0_t-th]-B*dth;
    
    tau_1_limit=50;
    tau_2_limit=30;
    tau_3_limit=20;
    
    if abs(tau(1))>tau_1_limit
        tau=[tau_1_limit*sign(tau(1));tau(2)];
    end
    if abs(tau(2))>tau_2_limit
        tau=[tau(1);tau_2_limit*sign(tau(2))];
    end
    
    if p.dof==3
       if abs(tau(3))>tau_3_limit 
          tau = [tau(1);tau(2);tau_3_limit*sign(tau(3))]; 
       end
    end
end