function [] = visualize_error_result(ekf_error, ukf_error, eif_error, ekf_abs_mean_error, ukf_abs_mean_error, eif_abs_mean_error)
    t=0:0.1:30;
    y_val = ones(length(t), 1);
    
    figure
    set(gcf,"Position",[600 100 1400 800])
    
    subplot(3,2,1)
    plot(t, abs(ekf_error.x), 'b--', LineWidth=1.5); hold on; grid on;
    plot(t, abs(ukf_error.x), 'g--', LineWidth=1.5);
    plot(t, abs(eif_error.x), 'k--', LineWidth=1.5); 
    title('X error')
    legend('EKF', 'UKF', 'EIF')
    
    subplot(3,2,3)
    plot(t, abs(ekf_error.y), 'b--', LineWidth=1.5); hold on; grid on;
    plot(t, abs(ukf_error.y), 'g--', LineWidth=1.5);
    plot(t, abs(eif_error.y), 'k--', LineWidth=1.5); 
    title('Y error')
    legend('EKF', 'UKF', 'EIF')
    
    subplot(3,2,5)
    plot(t, abs(rad2deg(ekf_error.yaw)), 'b--', LineWidth=1.5); hold on; grid on;
    plot(t, abs(rad2deg(ukf_error.yaw)), 'g--', LineWidth=1.5);
    plot(t, abs(rad2deg(eif_error.yaw)), 'k--', LineWidth=1.5); 
    title('Yaw error deg')
    legend('EKF', 'UKF', 'EIF')
    
    subplot(3,2,2)
    plot(t, y_val * ekf_abs_mean_error.x, 'b--', LineWidth=1.5); hold on; grid on;
    plot(t, y_val * ukf_abs_mean_error.x, 'g--', LineWidth=1.5);
    plot(t, y_val * eif_abs_mean_error.x, 'k--', LineWidth=1.5); 
    title('X mean error')
    legend('EKF', 'UKF', 'EIF')
    ylim([0 0.5])
    
    subplot(3,2,4)
    plot(t, y_val * ekf_abs_mean_error.y, 'b--', LineWidth=1.5); hold on; grid on;
    plot(t, y_val * ukf_abs_mean_error.y, 'g--', LineWidth=1.5);
    plot(t, y_val * eif_abs_mean_error.y, 'k--', LineWidth=1.5); 
    title('Y mean error')
    legend('EKF', 'UKF', 'EIF')
    ylim([0 0.5])
    
    subplot(3,2,6)
    plot(t, y_val * ekf_abs_mean_error.yaw, 'b--', LineWidth=1.5); hold on; grid on;
    plot(t, y_val * ukf_abs_mean_error.yaw, 'g--', LineWidth=1.5);
    plot(t, y_val * eif_abs_mean_error.yaw, 'k--', LineWidth=1.5); 
    title('Yaw mean error deg')
    legend('EKF', 'UKF', 'EIF')
    ylim([0 5])
end

