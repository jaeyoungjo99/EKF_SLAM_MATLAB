function [abs_mean_error] = calMeanError(filter_name, error)

    abs_mean_error.x = sum(abs(error.x)) / length(error.x);
    abs_mean_error.y = sum(abs(error.y)) / length(error.x);
    abs_mean_error.yaw = sum(abs(rad2deg(error.yaw))) / length(error.x);
    
    fprintf("\n***%s***\n", filter_name);
    fprintf("x abs mean error: %f , y abs mean error: %f , yaw abs mean error: %f \n", abs_mean_error.x, abs_mean_error.y, abs_mean_error.yaw);
end

