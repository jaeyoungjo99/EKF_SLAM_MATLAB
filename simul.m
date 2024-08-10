function [GT_val, sensor_val, input] = simul(position_noise, heading_noise)
    dt = 0.1;
    t = 0:dt:30;

    v = 10 + cos(t);
    yaw_rate = sin(t);

    x = zeros(length(t),1);
    y = zeros(length(t),1);
    yaw = zeros(length(t),1);

    sensor_val.x = zeros(length(t),1);
    sensor_val.y = zeros(length(t),1);
    sensor_val.yaw = zeros(length(t),1);

    for idx=1:length(t)
        if idx == 1
            yaw(idx) = yaw_rate(idx) * dt;

            sensor_val.x(idx) = x(idx) + normrnd(0, position_noise);
            sensor_val.y(idx) = y(idx) + normrnd(0, position_noise);
            sensor_val.yaw(idx) = yaw(idx) + normrnd(0, deg2rad(heading_noise));

            continue;
        end

        yaw(idx) = yaw(idx-1) + yaw_rate(idx) * dt;
        x(idx) = x(idx-1) + v(idx) * cos(yaw(idx)) * dt;
        y(idx) = y(idx-1) + v(idx) * sin(yaw(idx)) * dt;
        
        sensor_val.x(idx) = x(idx) + normrnd(0, position_noise);
        sensor_val.y(idx) = y(idx) + normrnd(0, position_noise);
        sensor_val.yaw(idx) = yaw(idx) + normrnd(0, deg2rad(heading_noise));
    end

    GT_val.x = x;
    GT_val.y = y;
    GT_val.yaw = yaw;

    input.vel = v;
    input.yaw_rate = yaw_rate;

    figure
    subplot(2,2,1) 
    plot(t,x);
    grid on; hold on;
    plot(t,sensor_val.x);
    title('x')
    legend('GT', 'Sensor')

    subplot(2,2,2)
    plot(t,y);
    grid on; hold on;
    plot(t,sensor_val.y);
    title('y')
    legend('GT', 'Sensor')

    subplot(2,2,3)
    plot(t,rad2deg(yaw));
    grid on; hold on;
    plot(t,rad2deg(sensor_val.yaw));
    title('yaw')
    legend('GT', 'Sensor')

    subplot(2,2,4)
    plot(x,y);
    grid on; hold on;
    plot(sensor_val.x,sensor_val.y);
    title('trajectory')
    legend('GT', 'Sensor')
end

