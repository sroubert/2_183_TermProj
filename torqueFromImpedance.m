%Gets torque values according to tau = K*(delta Theta) + B*(Theta_dot)

function tau = torqueFromImpedance(z,ti,t,th0,p)
    if p.dof==3
        th1=z(1);
        th2=z(2);
        th3=z(3);
        
    
        dth1=z(4);
        dth2=z(5);
        dth3=z(6);
        
        th=[th1;th2;th3];
        dth=[dth1;dth2;dth3];
        
        th0_t=[interp1(t,th0(:,1),ti);interp1(t,th0(:,2),ti),interp1(t,th0(:,3),ti)]; %approx th0 values at specified t
    end
    
    if p.dof==2
        th1=z(1);
        th2=z(2);
    
        dth1=z(3);
        dth2=z(4);
        
        th=[th1;th2];
        dth=[dth1;dth2;dth3];
    
        th0_t=[interp1(t,th0(:,1),ti);interp1(t,th0(:,2),ti)]; %approx th0 values at specified t
    end

    
    K=p.K;
    B=p.B;
    
    th0_t=[interp1(t,th0(:,1),ti);interp1(t,th0(:,2),ti)]; %approx th0 values at specified t
    
    tau=K*[th0_t-th]-B*dth;
end