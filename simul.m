function [GT_val, sensor_val, input] = simul(vel_noise, yaw_rate_noise, position_noise, heading_noise)
    % 이 함수는 주어진 position_noise와 heading_noise 값을 사용하여
    % 차량의 움직임을 시뮬레이션합니다. 실제 경로(GT_val)와 센서에서
    % 측정된 값(sensor_val)을 반환하며, 속도와 yaw_rate (input)도 반환합니다.

    dt = 0.1; % 시간 간격 설정 (초)
    t = 0:dt:30; % 시뮬레이션 시간 벡터 생성 (0초에서 30초까지)

    v = 10 + cos(t); % 차량의 속도: 10 m/s를 중심으로 진폭 1의 코사인 파형으로 변동
    yaw_rate = sin(t) + 10.0*pi/180.0; % 차량의 yaw rate: 사인 파형으로 변동

    % 차량의 초기 위치 및 각도를 저장할 벡터 생성
    x = zeros(length(t),1);
    y = zeros(length(t),1);
    yaw = zeros(length(t),1);

    % 센서에서 측정된 위치 및 각도를 저장할 벡터 생성
    sensor_val.x = zeros(length(t),1);
    sensor_val.y = zeros(length(t),1);
    sensor_val.yaw = zeros(length(t),1);

    % 차량의 이동을 시뮬레이션
    for idx = 1:length(t)
        if idx == 1
            % 첫 번째 시간 스텝에서 초기 yaw 값 설정
            yaw(idx) = yaw_rate(idx) * dt;

            % 첫 번째 시간 스텝에서 센서 측정값 계산
            sensor_val.x(idx) = x(idx) + normrnd(0, position_noise); % 위치 노이즈 추가
            sensor_val.y(idx) = y(idx) + normrnd(0, position_noise); % 위치 노이즈 추가
            sensor_val.yaw(idx) = yaw(idx) + normrnd(0, deg2rad(heading_noise)); % 각도 노이즈 추가

            continue; % 첫 번째 스텝 이후 계산을 계속하기 위해 루프 진행
        end

        % 두 번째 스텝부터 차량의 위치 및 yaw 값 계산
        yaw(idx) = yaw(idx-1) + yaw_rate(idx) * dt; % 이전 yaw 값에 yaw rate을 더함
        x(idx) = x(idx-1) + v(idx) * cos(yaw(idx)) * dt; % 이전 위치에서 속도와 yaw 각도를 통해 새로운 x 좌표 계산
        y(idx) = y(idx-1) + v(idx) * sin(yaw(idx)) * dt; % 이전 위치에서 속도와 yaw 각도를 통해 새로운 y 좌표 계산
        
        % 센서 측정값 계산
        sensor_val.x(idx) = x(idx) + normrnd(0, position_noise); % 위치 노이즈 추가
        sensor_val.y(idx) = y(idx) + normrnd(0, position_noise); % 위치 노이즈 추가
        sensor_val.yaw(idx) = yaw(idx) + normrnd(0, deg2rad(heading_noise)); % 각도 노이즈 추가
    end



    % 결과 구조체에 실제 값 저장
    GT_val.x = x;
    GT_val.y = y;
    GT_val.yaw = yaw;

    % 결과 구조체에 입력값 저장
    input.vel = v;
    input.yaw_rate = yaw_rate;
    for idx = 1:length(t)
        input.vel(idx) = input.vel(idx) + normrnd(0,vel_noise);
        input.yaw_rate(idx) = input.yaw_rate(idx) + normrnd(0,yaw_rate_noise);
    end

    % 시뮬레이션 결과를 플롯으로 시각화
    figure
    
    % x 좌표 플롯
    subplot(2,2,1) 
    plot(t, x); % 실제 x 값 플롯
    grid on; hold on;
    plot(t, sensor_val.x); % 센서 측정된 x 값 플롯
    title('x')
    legend('GT', 'Sensor')

    % y 좌표 플롯
    subplot(2,2,2)
    plot(t, y); % 실제 y 값 플롯
    grid on; hold on;
    plot(t, sensor_val.y); % 센서 측정된 y 값 플롯
    title('y')
    legend('GT', 'Sensor')

    % yaw 값 플롯
    subplot(2,2,3)
    plot(t, rad2deg(yaw)); % 실제 yaw 값 (도 단위로 변환 후) 플롯
    grid on; hold on;
    plot(t, rad2deg(sensor_val.yaw)); % 센서 측정된 yaw 값 플롯
    title('yaw')
    legend('GT', 'Sensor')

    % 전체 경로 (x vs y) 플롯
    subplot(2,2,4)
    plot(x, y); % 실제 경로 플롯
    grid on; hold on;
    plot(sensor_val.x, sensor_val.y); % 센서 측정된 경로 플롯
    title('trajectory')
    legend('GT', 'Sensor')
end
