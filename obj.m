function J = obj(X, time, x_com_0, x_com_dot_0, y_com_0, y_com_dot_0, phi_0, phi_dot_0, xo, yo, ro, L)

    h = time(2)-time(1);
    n = length(time);
    z = X(1);
    x_list = [x_com_0, x_com_dot_0, y_com_0, y_com_dot_0, phi_0 , phi_dot_0, z];
    u_list = [X(2), X(3), X(4), X(5)];
    
    % n_vars = 10*(n-1) + 4 + 1;
    n_vars = length(X);
    
    i = 6;
    while i <= n_vars
        x_list = [x_list; [X(i), X(i+1), X(i+2), X(i+3), X(i+4), X(i+5), z]];
        u_list = [u_list; [X(i+6), X(i+7), X(i+8), X(i+9)]];
        i = i + 10;
    end

    obj_sum = 0;
    i = 1;
    while i <= length(x_list(:, 1))-1
        x_com_temp = x_list(i,1);
        y_com_temp = x_list(i,3);
        i = i+1;
        x_com_1_temp = x_list(i,1);
        y_com_1_tmep = x_list(i,3);
        dist = sqrt((x_com_temp-xo)^2 + (y_com_temp-yo)^2);
        dist1 = sqrt((x_com_1_temp-xo)^2 + (y_com_1_tmep-yo)^2);
        obj_sum = obj_sum + z * (dist+dist1)*h;

    end
    
    x = x_list(end, :);
    x_com = x(1);
    y_com = x(3);

    % J = 0.5 * obj_sum;
    J = 2*sqrt((x_com-xo)^2 + (y_com-yo)^2) + z;
    
end