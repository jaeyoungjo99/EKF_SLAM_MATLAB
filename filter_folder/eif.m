function [estimated_state, error] = eif(P0, Q, R, GT_val, sensor_val, input)
    
    STATE_ORDER = 3;
    dt = 0.1;
    total_steps = length(sensor_val.x);

    information_vector_pri = zeros([STATE_ORDER, total_steps]);
    information_vector_pos = zeros([STATE_ORDER, total_steps]);
    information_matrix_pri = zeros([STATE_ORDER, STATE_ORDER, total_steps]);
    information_matrix_pos = zeros([STATE_ORDER, STATE_ORDER, total_steps]);

    x_pri = zeros([STATE_ORDER, total_steps]);
    x_pos = zeros([STATE_ORDER, total_steps]);

    error.x = zeros(total_steps,1);
    error.y = zeros(total_steps,1);
    error.yaw = zeros(total_steps,1);

    sensor_meas = zeros([3, total_steps]);
    yaw_rate = input.yaw_rate;
    vel = input.vel;

    H_jacob = [1 0 0
               0 1 0
               0 0 1];

    inv_R = inv(R);

    isFirstStep = true;

    for idxSim = 1: total_steps

        sensor_meas(1,idxSim) = sensor_val.x(idxSim);
        sensor_meas(2,idxSim) = sensor_val.y(idxSim);
        sensor_meas(3,idxSim) = sensor_val.yaw(idxSim);
                
        if isFirstStep
            % 초기 정보 행렬 및 정보 벡터 설정
            information_matrix_pri(:, :, idxSim) = inv(P0); % 초기 정보 행렬
            information_matrix_pos(:, :, idxSim) = inv(P0); % 초기 정보 행렬

            information_vector_pri(:, idxSim) = information_matrix_pri(:,:,idxSim) * sensor_meas(:, idxSim); % 초기 정보 벡터
            information_vector_pos(:, idxSim) = information_matrix_pos(:,:,idxSim) * sensor_meas(:, idxSim); % 초기 정보 벡터
            
            x_pri(:, idxSim) = sensor_meas(:, idxSim);
            x_pos(:, idxSim) = sensor_meas(:, idxSim);

            isFirstStep = false;
            continue;
        end

        F_jacob = Fjacob(x_pos(:,idxSim-1), vel(idxSim-1), dt);
        
        x_pri(:, idxSim) = fk(x_pos(:, idxSim - 1), vel(idxSim), yaw_rate(idxSim), dt);

        information_matrix_pri(:,:,idxSim) = inv(F_jacob * information_matrix_pos(:,:,idxSim-1) * F_jacob' + Q);
        information_vector_pri(:, idxSim) = information_matrix_pri(:,:,idxSim) * x_pri(:,idxSim);
        
        information_matrix_pos(:,:,idxSim) = information_matrix_pri(:,:,idxSim) + H_jacob' * inv_R * H_jacob;
        information_vector_pos(:, idxSim) = information_vector_pri(:,idxSim) + H_jacob' * inv_R * (sensor_meas(:, idxSim) - hk(x_pri(:, idxSim)) + H_jacob * x_pri(:,idxSim));

        x_pos(:, idxSim) = inv(information_matrix_pos(:,:,idxSim)) * information_vector_pos(:, idxSim);
        
    end

    estimated_state.x = x_pos(1, :)';
    estimated_state.y = x_pos(2, :)';
    estimated_state.yaw = x_pos(3, :)';

    estimated_error = calError(GT_val, estimated_state);
    sensor_error = calError(GT_val, sensor_val);

    error = estimated_error;

    visualize_result('EIF', GT_val, sensor_val, estimated_state, estimated_error, sensor_error)

end

%%

function F = Fjacob(xp,vel,dt)

    yaw = xp(3);
    
    F = [1 0 -vel*dt*sin(yaw)
         0 1 vel*dt*cos(yaw)
         0 0 1];
end

function f = fk (xp, vel, yaw_rate, dt)

    x = xp(1);
    y = xp(2);

    if xp(3) > 2 * pi
        xp(3) = xp(3) - 2 * pi;
    elseif xp(3) < -2 * pi
        xp(3) = xp(3) + 2 * pi;
    end

    yaw = xp(3);
    
    f = [x + vel * dt * cos(yaw)
         y + vel * dt * sin(yaw)
         yaw + dt * yaw_rate];
end

function h = hk (xp)

    x = xp(1);
    y = xp(2);
    yaw = xp(3);
    
    h = [x
         y
         yaw];
end
