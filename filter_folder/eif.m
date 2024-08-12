% Extended Information Filter (EIF)
% 이 함수는 Extended Information Filter를 구현하여 추정된 상태와 오차를 계산합니다.
% EIF는 Kalman 필터의 정보 형태로, 정보 벡터와 정보 행렬을 사용하여 상태를 추정합니다.
% 상태 벡터를 정보 벡터로, 공분산 행렬을 정보 행렬로 변환하여 필터링 과정을 수행합니다.
%
% 입력 파라미터:
% P0: 초기 상태 공분산 행렬
% Q: 프로세스 노이즈 공분산 행렬
% R: 측정 노이즈 공분산 행렬
% GT_val: 실제 값 (Ground Truth)
% sensor_val: 센서 측정 값
% input: 입력 값 (속도와 yaw rate)
%
% 출력:
% estimated_state: 추정된 상태
% error: 추정된 상태와 실제 값 간의 오차

function [estimated_state, error] = eif(P0, Q, R, GT_val, sensor_val, input)
    
    STATE_ORDER = 3; % 상태 변수의 개수 (x, y, yaw)
    dt = 0.1; % 시간 간격 (초)
    total_steps = length(sensor_val.x); % 전체 시뮬레이션 스텝 수

    % 정보 벡터와 정보 행렬을 저장할 배열 초기화
    information_vector_pri = zeros([STATE_ORDER, total_steps]);
    information_vector_pos = zeros([STATE_ORDER, total_steps]);
    information_matrix_pri = zeros([STATE_ORDER, STATE_ORDER, total_steps]);
    information_matrix_pos = zeros([STATE_ORDER, STATE_ORDER, total_steps]);

    % 추정된 상태를 저장할 배열 초기화
    x_pri = zeros([STATE_ORDER, total_steps]);
    x_pos = zeros([STATE_ORDER, total_steps]);

    % 오차를 저장할 배열 초기화
    error.x = zeros(total_steps,1);
    error.y = zeros(total_steps,1);
    error.yaw = zeros(total_steps,1);

    % 센서 측정값을 저장할 배열 초기화
    sensor_meas = zeros([3, total_steps]);
    yaw_rate = input.yaw_rate;
    vel = input.vel;

    H_jacob = [1 0 0
               0 1 0
               0 0 1];

    inv_R = inv(R);

    isFirstStep = true;

    % EIF 루프
    for idxSim = 1: total_steps
        % 현재 시뮬레이션 스텝의 센서 측정값을 저장
        sensor_meas(1,idxSim) = sensor_val.x(idxSim);
        sensor_meas(2,idxSim) = sensor_val.y(idxSim);
        sensor_meas(3,idxSim) = sensor_val.yaw(idxSim);
        
        if isFirstStep
            % 초기 정보 행렬 및 정보 벡터 설정
            information_matrix_pri(:, :, idxSim) = inv(P0); % 초기 정보 행렬
            information_matrix_pos(:, :, idxSim) = inv(P0); % 초기 정보 행렬

            % 초기 정보 벡터
            information_vector_pri(:, idxSim) = information_matrix_pri(:,:,idxSim) * sensor_meas(:, idxSim);
            information_vector_pos(:, idxSim) = information_matrix_pos(:,:,idxSim) * sensor_meas(:, idxSim);
            
            % 초기 상태 벡터 설정
            x_pri(:, idxSim) = sensor_meas(:, idxSim);
            x_pos(:, idxSim) = sensor_meas(:, idxSim);

            isFirstStep = false;
            continue;
        end

        % 상태 전이 행렬 F의 야코비안 계산
        F_jacob = Fjacob(x_pos(:,idxSim-1), vel(idxSim-1), dt);
        
        % 예측 단계: 정보 벡터 및 정보 행렬 업데이트
        x_pri(:, idxSim) = fk(x_pos(:, idxSim - 1), vel(idxSim), yaw_rate(idxSim), dt);

        % Information matrix 예측 단계:
        % \Lambda_{k|k-1} = (F_k \Lambda_{k-1|k-1} F_k^T + Q_k)^{-1}
        information_matrix_pri(:,:,idxSim) = inv(F_jacob * inv(information_matrix_pos(:,:,idxSim-1)) * F_jacob' + Q);

        % Information vector 예측 단계:
        % \xi_{k|k-1} = \Lambda_{k|k-1} * \hat{x}_{k|k-1}
        information_vector_pri(:, idxSim) = information_matrix_pri(:,:,idxSim) * x_pri(:,idxSim);
        
        % 정보 행렬 갱신 단계:
        % \Lambda_{k|k} = \Lambda_{k|k-1} + H_k^T R^{-1} H_k
        information_matrix_pos(:,:,idxSim) = information_matrix_pri(:,:,idxSim) + H_jacob' * inv_R * H_jacob;

        % 정보 벡터 갱신 단계:
        % \xi_{k|k} = \xi_{k|k-1} + H_k^T R^{-1} (z_k - h(\hat{x}_{k|k-1}) + H_k * \hat{x}_{k|k-1})
        information_vector_pos(:, idxSim) = information_vector_pri(:,idxSim) + ...
                H_jacob' * inv_R * (yk(sensor_meas(:, idxSim),hk(x_pri(:, idxSim))) + H_jacob * x_pri(:,idxSim));

        % 상태 벡터 갱신:
        % \hat{x}_{k|k} = \Lambda_{k|k}^{-1} \xi_{k|k}
        x_pos(:, idxSim) = inv(information_matrix_pos(:,:,idxSim)) * information_vector_pos(:, idxSim);
    end

    % 최종 추정된 상태를 구조체에 저장
    estimated_state.x = x_pos(1, :)';
    estimated_state.y = x_pos(2, :)';
    estimated_state.yaw = x_pos(3, :)';

    % 추정된 오차 계산
    estimated_error = calError(GT_val, estimated_state);
    sensor_error = calError(GT_val, sensor_val);

    % 최종 오차를 구조체로 반환
    error = estimated_error;

    % 결과 시각화
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
