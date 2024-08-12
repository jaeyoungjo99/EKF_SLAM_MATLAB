function [] = visualize_result(figure_name, GT, sensor, estimated_state, estimated_error, sensor_error)
    t = 0:0.1:30;

    figure('Name', figure_name)

    subplot(4,2,[1 2])
    plot(GT.x, GT.y, 'g--', LineWidth=1.5);
    hold on; grid on;
    plot(sensor.x, sensor.y, 'r--', LineWidth=1.5);
    plot(estimated_state.x, estimated_state.y, 'b--', LineWidth=1.5);
    legend('GT', 'sensor', 'estimate')
    title('trajectory')

    subplot(4,2,3)
    plot(t, GT.x, 'g--', LineWidth=1.5);
    hold on; grid on;
    plot(t, sensor.x, 'r--', LineWidth=1.5);
    plot(t, estimated_state.x, 'b--', LineWidth=1.5);
    legend('GT', 'sensor', 'estimate')
    title('x')

    subplot(4,2,5)
    plot(t, GT.y, 'g--', LineWidth=1.5);
    hold on; grid on;
    plot(t, sensor.y, 'r--', LineWidth=1.5);
    plot(t, estimated_state.y, 'b--', LineWidth=1.5);
    legend('GT', 'sensor', 'estimate')
    title('y')

    subplot(4,2,7)
    plot(t, rad2deg(GT.yaw), 'g--', LineWidth=1.5);
    hold on; grid on;
    plot(t, rad2deg(sensor.yaw), 'r--', LineWidth=1.5);
    plot(t, rad2deg(estimated_state.yaw), 'b--', LineWidth=1.5);
    legend('GT', 'sensor', 'estimate')
    title('yaw')

    subplot(4,2,4)
    plot(t, estimated_error.x);
    hold on; grid on;
    plot(t, sensor_error.x);
    legend('estimated error', 'sensor error')
    title('x error')

    subplot(4,2,6)
    plot(t, estimated_error.y);
    hold on; grid on;
    plot(t, sensor_error.y);
    legend('estimated error', 'sensor error')
    title('y error')

    subplot(4,2,8)
    plot(t, rad2deg(estimated_error.yaw));
    hold on; grid on;
    plot(t, rad2deg(sensor_error.yaw));
    legend('estimated error', 'sensor error')
    title('yaw error deg')
    
end