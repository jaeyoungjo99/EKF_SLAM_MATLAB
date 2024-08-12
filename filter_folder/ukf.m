function [estimated_state, error] = ukf(P0, Q, R, GT_val, sensor_val, input, alpha, beta, kappa)
    
    STATE_ORDER = 3;
    dt = 0.1;
    total_steps = length(sensor_val.x);

    x_posteriori = zeros([STATE_ORDER, total_steps]);
    P_posteriori = zeros([STATE_ORDER, STATE_ORDER, total_steps]);

    error.x = zeros(total_steps,1);
    error.y = zeros(total_steps,1);
    error.yaw = zeros(total_steps,1);

    sensor_meas = zeros([3, total_steps]);
    yaw_rate = input.yaw_rate;
    vel = input.vel;

    isFirstStep = true;

    for idxSim = 1: total_steps

        sensor_meas(1,idxSim) = sensor_val.x(idxSim);
        sensor_meas(2,idxSim) = sensor_val.y(idxSim);
        sensor_meas(3,idxSim) = sensor_val.yaw(idxSim);
                
        if isFirstStep
            x_posteriori(:, idxSim) = sensor_meas(:, idxSim);
            P_posteriori(:, :, idxSim) = P0;
            isFirstStep = false;
            continue;
        end

        % Generate sigma points
        [sigma_points, Wm, Wc] = generateSigmaPoints(x_posteriori(:, idxSim-1), P_posteriori(:, :, idxSim-1), alpha, beta, kappa);

        % Predict sigma points (3(state num) x 7(sigma num, 2n+1))
        predicted_sigma_points = zeros(size(sigma_points));
        for i = 1:size(sigma_points, 2)
            predicted_sigma_points(:, i) = fk(sigma_points(:, i), vel(idxSim), yaw_rate(idxSim), dt);
        end

        % Predict state mean and covariance
        % u_t^bar
        x_priori = predicted_sigma_points * Wm';

        % Sigma_t = Sum(w_c^i*(X_t^i - u_t^bar) * (X_t^i - u_t^bar)^T) + R_t
        P_priori = Q;
        for i = 1:size(predicted_sigma_points, 2)
            P_priori = P_priori + Wc(i) * (predicted_sigma_points(:, i) - x_priori) * (predicted_sigma_points(:, i) - x_priori)';
        end

        % Transform sigma points into measurement space
        % Z_t^bar
        predicted_meas_sigma_points = zeros(3, size(predicted_sigma_points, 2));
        for i = 1:size(predicted_sigma_points, 2)
            predicted_meas_sigma_points(:, i) = hk(predicted_sigma_points(:, i));
        end

        % Predicted measurement mean and covariance
        % Z_t^bar = h(X_t^bar)
        z_priori = predicted_meas_sigma_points * Wm';

        % Sigma_t = Sum(w_c^i*(Z_t^i - z_t^hat) * (Z_t^i - z_t^hat)^T) + Q_t
        S = R;
        for i = 1:size(predicted_meas_sigma_points, 2)            
            S = S + Wc(i) * (predicted_meas_sigma_points(:, i) - z_priori) * (predicted_meas_sigma_points(:, i) - z_priori)';
        end

        % Cross-covariance
        Pxz = zeros(STATE_ORDER, 3);
        for i = 1:size(predicted_sigma_points, 2)
            Pxz = Pxz + Wc(i) * (predicted_sigma_points(:, i) - x_priori) * (predicted_meas_sigma_points(:, i) - z_priori)';
        end

        
        % Kalman gain
        K = Pxz / S;

        % Update state mean and covariance
        x_posteriori(:, idxSim) = x_priori + K * yk(sensor_meas(:, idxSim),z_priori);
        P_posteriori(:, :, idxSim) = P_priori - K * S * K';

    end

    estimated_state.x = x_posteriori(1, :)';
    estimated_state.y = x_posteriori(2, :)';
    estimated_state.yaw = x_posteriori(3, :)';

    estimated_error = calError(GT_val, estimated_state);
    sensor_error = calError(GT_val, sensor_val);

    error = estimated_error;

    visualize_result('UKF', GT_val, sensor_val, estimated_state, estimated_error, sensor_error)

end

%%

function [sigma_points, Wm, Wc] = generateSigmaPoints(x, P, alpha, beta, kappa)
    % UKF에서 사용하는 시그마 포인트 생성
    L = length(x);
    lambda = alpha^2 * (L + kappa) - L;
    sigma_points = zeros(L, 2 * L + 1);
    Wm = zeros(1, 2 * L + 1);
    Wc = Wm;

    sigma_points(:, 1) = x;
    Wm(1) = lambda / (L + lambda);
    Wc(1) = Wm(1) + (1 - alpha^2 + beta);

    U = chol((L + lambda) * P);
    for i = 1:L
        sigma_points(:, i+1) = x + U(i, :)';
        sigma_points(:, i+L+1) = x - U(i, :)';
        Wm(i+1) = 1 / (2 * (L + lambda));
        Wm(i+L+1) = Wm(i+1);
        Wc(i+1) = Wm(i+1);
        Wc(i+L+1) = Wc(i+1);
    end
end

function f = fk (xp, vel, yaw_rate, dt)

    x = xp(1);
    y = xp(2);
    yaw = xp(3);
    
    f = [x + vel *dt*cos(yaw)
         y + vel *dt*sin(yaw)
         yaw + dt*yaw_rate];
end

function h = hk (xp)

    x = xp(1);
    y = xp(2);
    yaw = xp(3);
    
    h = [x
         y
         yaw];
end

function yk = yk (z, h)
    
    yk = z - h;

    % Angle Normalization
    while yk(3) > pi
        yk(3) = yk(3) - 2 * pi;
    end
    while yk(3) < -pi
        yk(3) = yk(3) + 2 * pi;
    end

end
