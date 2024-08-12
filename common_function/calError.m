function [error] = calError(GT_val, target)
    error.x = GT_val.x - target.x;
    error.y = GT_val.y - target.y;
    error.yaw = GT_val.yaw - target.yaw;
end