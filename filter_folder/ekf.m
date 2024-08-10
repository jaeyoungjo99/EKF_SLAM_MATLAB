% Extended Kalman Filter
function [estimated_state, error] = ekf(P0, Q, R, GT_val, sensor_val, input)
    
    STATE_ORDER = 3;
    dt = 0.1;
    total_steps = length(sensor_val.x);

    x_priori = zeros([STATE_ORDER, total_steps]);
    x_posteriori = zeros([STATE_ORDER, total_steps]);

    P_priori = zeros([STATE_ORDER, STATE_ORDER, total_steps]);
    P_posteriori = zeros([STATE_ORDER, STATE_ORDER, total_steps]);

    error.x = zeros(total_steps,1);
    error.y = zeros(total_steps,1);
    error.yaw = zeros(total_steps,1);

    sensor_meas = zeros([3, total_steps]);
    yaw_rate = input.yaw_rate;
    vel = input.vel;
    
    H_jacob = [1 0 0
               0 1 0
               0 0 1];
    
    isFirstStep = true;
    
    for idxSim = 1: total_steps

        sensor_meas(1,idxSim) = sensor_val.x(idxSim);
        sensor_meas(2,idxSim) = sensor_val.y(idxSim);
        sensor_meas(3,idxSim) = sensor_val.yaw(idxSim);
                
        if isFirstStep  == true

            x_priori(:, idxSim) = sensor_meas(:, idxSim);
            P_priori(:, :, idxSim) = P0;

            x_posteriori(:, idxSim) = sensor_meas(:, idxSim);
            P_posteriori(:, :, idxSim) = P0;

            isFirstStep = false;
            continue;
        end


        F_jacob = Fjacob(x_posteriori(:,idxSim-1), vel(idxSim-1), dt);

        P_priori(:, :, idxSim) = F_jacob * P_posteriori(:, :, idxSim - 1) * F_jacob' + Q;

        x_priori(:, idxSim) = fk(x_posteriori(:, idxSim - 1), vel(idxSim), yaw_rate(idxSim), dt);
        
        K = P_priori(:, :, idxSim) * H_jacob' * inv(H_jacob * P_priori(:, :, idxSim) * H_jacob' + R);
        x_posteriori(:, idxSim) = x_priori(:, idxSim) + K * (sensor_meas(:, idxSim) - hk(x_priori(:, idxSim)));
        P_posteriori(:, :, idxSim) = (eye(STATE_ORDER) - K * H_jacob) * P_priori(:, :, idxSim);

    end

    estimated_state.x = x_posteriori(1, :)';
    estimated_state.y = x_posteriori(2, :)';
    estimated_state.yaw = x_posteriori(3, :)';

    estimated_error = calError(GT_val, estimated_state);
    sensor_error = calError(GT_val, sensor_val);
    
    error = estimated_error;
    
    visualize_result('EKF', GT_val, sensor_val, estimated_state, estimated_error, sensor_error)

end

%%

function F = Fjacob(xp,vel,dt)

    yaw = xp(3);
    
    F = [1 0 -vel*dt*sin(yaw)
         0 1 vel*dt*cos(yaw)
         0 0 1];
end

function f = fk (xp,vel,yaw_rate,dt)

    x = xp(1);
    y = xp(2);

    if xp(3) > 2 * pi
        xp(3) = xp(3) - 2 * pi;
    elseif xp(3) < -2 * pi
        xp(3) = xp(3) + 2 * pi;
    end

    yaw = xp(3);
    
    f = [x + vel *dt*cos(yaw)
         y + vel *dt*sin(yaw)
         yaw + dt*yaw_rate];
end

function h = hk (xp)

    x = xp(1);
    y = xp(2);
    yaw = xp(3);
    
    h= [x
        y
        yaw];
end


