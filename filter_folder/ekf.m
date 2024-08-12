% Extended Kalman Filter
% 이 함수는 Extended Kalman Filter (EKF)를 구현하여 추정된 상태와 오차를 계산합니다.
% 입력 파라미터:
% P0: 초기 상태 공분산 행렬
% Q: 프로세스 노이즈 공분산 행렬
% R: 측정 노이즈 공분산 행렬
% GT_val: 실제 값 (Ground Truth)
% sensor_val: 센서 측정 값
% input: 입력 값 (속도와 yaw rate)
% 출력:
% estimated_state: 추정된 상태
% error: 추정된 상태와 실제 값 간의 오차

function [estimated_state, error] = ekf(P0, Q, R, GT_val, sensor_val, input)
    STATE_ORDER = 3; % 상태 변수의 개수 (x, y, yaw)
    dt = 0.1; % 시간 간격 (초)
    total_steps = length(sensor_val.x); % 전체 시뮬레이션 스텝 수

    % 추정된 상태와 공분산 행렬을 저장할 배열 초기화
    x_priori = zeros([STATE_ORDER, total_steps]);
    x_posteriori = zeros([STATE_ORDER, total_steps]);

    P_priori = zeros([STATE_ORDER, STATE_ORDER, total_steps]);
    P_posteriori = zeros([STATE_ORDER, STATE_ORDER, total_steps]);

    % 오차를 저장할 배열 초기화
    error.x = zeros(total_steps,1);
    error.y = zeros(total_steps,1);
    error.yaw = zeros(total_steps,1);

    % 센서 측정값을 저장할 배열 초기화
    sensor_meas = zeros([3, total_steps]);
    yaw_rate = input.yaw_rate;
    vel = input.vel;
    
    % 측정 행렬 H 초기화 (여기서는 단위 행렬)
    H_jacob = [1 0 0
               0 1 0
               0 0 1];
    
    isFirstStep = true; % 첫 번째 스텝인지 확인하기 위한 플래그
    
    % EKF 루프
    for idxSim = 1: total_steps
        % 현재 시뮬레이션 스텝의 센서 측정값을 저장
        sensor_meas(1,idxSim) = sensor_val.x(idxSim);
        sensor_meas(2,idxSim) = sensor_val.y(idxSim);
        sensor_meas(3,idxSim) = sensor_val.yaw(idxSim);
        
        if isFirstStep  == true
            % 첫 번째 스텝에서는 측정값을 그대로 초기 상태로 사용
            x_priori(:, idxSim) = sensor_meas(:, idxSim);
            P_priori(:, :, idxSim) = P0;

            x_posteriori(:, idxSim) = sensor_meas(:, idxSim);
            P_posteriori(:, :, idxSim) = P0;

            isFirstStep = false; % 첫 번째 스텝 완료
            continue; % 다음 스텝으로 이동
        end

        % 상태 전이 행렬 F 계산
        F_jacob = Fjacob(x_posteriori(:,idxSim-1), vel(idxSim-1), dt);

        % 예측 단계: 공분산 행렬 업데이트
        P_priori(:, :, idxSim) = F_jacob * P_posteriori(:, :, idxSim - 1) * F_jacob' + Q;

        % 예측 단계: 상태 업데이트
        x_priori(:, idxSim) = fk(x_posteriori(:, idxSim - 1), vel(idxSim), yaw_rate(idxSim), dt);
        
        % 칼만 이득 계산
        K = P_priori(:, :, idxSim) * H_jacob' * inv(H_jacob * P_priori(:, :, idxSim) * H_jacob' + R);

        % 갱신 단계: 상태 및 공분산 행렬 업데이트
        % x_posteriori(:, idxSim) = x_priori(:, idxSim) + K * (sensor_meas(:, idxSim) - hk(x_priori(:, idxSim)));
        x_posteriori(:, idxSim) = x_priori(:, idxSim) + K * yk(sensor_meas(:, idxSim), hk(x_priori(:, idxSim)));
        P_posteriori(:, :, idxSim) = (eye(STATE_ORDER) - K * H_jacob) * P_priori(:, :, idxSim);
    end

    % 최종 추정된 상태를 구조체에 저장
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

