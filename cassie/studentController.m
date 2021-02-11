function tau = studentController(t, s, model, params)
    %% Extract generalized coordinates and velocities
    q = s(1 : model.n);
    dq = s(model.n+1 : 2*model.n);

    %% Control Scheme
    [p1, p2, p3, p4] = computeFootPositions(q, model);
    g = 9.81;
    if p1(3) > 0.05 || p2(3) > 0.05 || p3(3) > 0.05 || p4(3) > 0.05
        kp = 500 ;
        kd = 100 ;
        x0 = getInitialState(model);
        q0 = x0(1:model.n) ;
        tau = -kp*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd*dq(model.actuated_idx);
    else
        kpLat = diag([10000, 10000,6325]);
        kdLat = diag([775, 830, 700]);

        kpRot = diag([4000,2350,6000]);
        kdRot = diag([300,230,325]);
        
        % Find stabilizing wrench @ CoM
        x0 = getInitialState(model);
        q0 = x0(1:model.n);
        q0(3) = 0.8894; 
        dq0 = zeros(20, 1);

        rpy0 = [q0(6); q0(5); q0(4)];
        rpy = [q(6); q(5); q(4)];
        drpy = [dq(6); dq(5); dq(4)];
        [r_com0, v_com0] = computeComPosVel(q0, dq0, model);
        [r_com, v_com] = computeComPosVel(q, dq, model);

        % Rotation matrices
        RzD = Rz(rpy0(3));
        RyD = Ry(rpy0(2));
        RxD = Rx(rpy0(1));
        Rd = RzD*RyD*RxD;
        RzCurr = Rz(rpy(3));
        RyCurr = Ry(rpy(2));
        RxCurr = Rx(rpy(1));
        RCurr = RzCurr*RyCurr*RxCurr;

        % Finding rotational error term
        Rdb = Rd'*RCurr;
        quatRdb = rotm2quat(Rdb);
        scal = quatRdb(1);
        vec = quatRdb(2:4)';
        rotTau = -2*(scal*eye(3) + hat(vec))*kpRot*vec;
        tauD = RCurr*(rotTau - kdRot*drpy);

        e = r_com - r_com0;
        de = v_com;

        Fd = -kpLat*e - kdLat*de + [0; 0; model.M*g];
        wrenchD = [Fd; tauD];

        % Find contact forces (optimization problem)
        [p1, p2, p3, p4] = computeFootPositions(s, model);
        r1 = p1 - r_com;
        r2 = p2 - r_com;
        r3 = p3 - r_com;
        r4 = p4 - r_com;

        r1hat = hat(r1);
        r2hat = hat(r2);
        r3hat = hat(r3);
        r4hat = hat(r4);
        Gc = [eye(3) eye(3) eye(3) eye(3); r1hat r2hat r3hat r4hat];

        % Compute H matrix
        mu = 0.8;
        v1 = [-sin(atan(mu)); 0; cos(atan(mu))];
        v2 = [sin(atan(mu)); 0; cos(atan(mu))];
        v3 = [0; -sin(atan(mu)); cos(atan(mu))];
        v4 = [0; sin(atan(mu)); cos(atan(mu))];
        norm1 = hat(v1)*v3;
        norm2 = hat(v4)*v1;
        norm3 = hat(v3)*v2;
        norm4 = hat(v2)*v4;

        N = [norm1 norm2 norm3 norm4];

        % Compute Aeg
        zeroMat = zeros(3, 4);
        nMat = [N zeroMat zeroMat zeroMat; ...
                zeroMat N zeroMat zeroMat; ...
                zeroMat zeroMat N zeroMat; ...
                zeroMat zeroMat zeroMat N];

        params.G = Gc;
        params.b = wrenchD;
        params.n = nMat;
        params.idMat1 = [eye(3) zeros(3)];
        params.idMat2 = [zeros(3) eye(3)];
        params.lambda1 = 1;
        params.lambda2 = 0.1;
        params.lambda3 = 0.01;
        settings.verbose = 0;
        [vars, status] = csolve(params, settings);
        sigma = vars.sig;
        Fc = nMat * sigma;
        
        % Find motor torques
        [J1f, J1b, J2f, J2b] = computeFootJacobians(s,model);
        tau_ =       - ((J1f)'*[zeros(3,1); Fc(1:3)] + ...
                    (J1b)'*[zeros(3,1); Fc(4:6)] + ...
                    (J2f)'*[zeros(3,1); Fc(7:9)] + ...
                    (J2b)'*[zeros(3,1); Fc(10:12)]) ; 
        tau = tau_(7:end); 
    end
    
end

function rotZ = Rz(theta)
    rotZ = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
end

function rotY = Ry(theta)
    rotY = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
end

function rotX = Rx(theta)
    rotX = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
end

function hatVec = hat(v)
    hatVec = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end