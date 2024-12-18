function [c, ceq] = nonlcon(X, time, x_com_0, x_com_dot_0, y_com_0, y_com_dot_0, phi_0, phi_dot_0, xo, yo, ro, L)
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
    
    c = [];
    ceq = [];

    X_1 = x_list(1,:)';
    U_1 = u_list(1,:)';
    X_1_dot = system_dynamics(X_1, U_1)';

    x_com_temp = X_1(1);
    x_com_dot_temp = X_1(2);
    y_com_temp = X_1(3);
    y_com_dot_temp = X_1(4);
    phi_temp = X_1(5);
    phi_dot_temp = X_1(6);

    x_com_ddot_temp = X_1_dot(2);
    y_com_ddot_temp = X_1_dot(4);
    phi_ddot_temp = X_1_dot(6);

    x1_temp = x_com_temp - L/2*cos(phi_temp);
    y1_temp = y_com_temp - L/2*sin(phi_temp);
    x2_temp = x_com_temp + L/2*cos(phi_temp);
    y2_temp = y_com_temp + L/2*sin(phi_temp);

    c4 = ro^2 + 3 - (x1_temp-xo)^2 - (y1_temp-yo)^2;
    c5 = ro^2 + 3 - (x2_temp-xo)^2 - (y2_temp-yo)^2;

    c = [c, [c4, c5]];
    
    for i = 1:length(x_list(:, 1))-1
        xk = x_list(i,:)';
        xk1 = x_list(i+1,:)';
        uk = u_list(i,:)';
        uk1 = u_list(i+1,:)';
    
        err = error_const(xk, xk1, uk, uk1, h)';
        ceq = [ceq, err];

        X_1 = xk1;
        U_1 = uk1;
        X_1_dot = system_dynamics(X_1, U_1)';

        x_com_temp = X_1(1);
        x_com_dot_temp = X_1(2);
        y_com_temp = X_1(3);
        y_com_dot_temp = X_1(4);
        phi_temp = X_1(5);
        phi_dot_temp = X_1(6);
    
        x_com_ddot_temp = X_1_dot(2);
        y_com_ddot_temp = X_1_dot(4);
        phi_ddot_temp = X_1_dot(6);
    
        x1_temp = x_com_temp - L/2*cos(phi_temp);
        y1_temp = y_com_temp - L/2*sin(phi_temp);
        x2_temp = x_com_temp + L/2*cos(phi_temp);
        y2_temp = y_com_temp + L/2*sin(phi_temp);
    
        c4 = ro^2 + 3 - (x1_temp-xo)^2 - (y1_temp-yo)^2;
        c5 = ro^2 + 3 - (x2_temp-xo)^2 - (y2_temp-yo)^2;
    
        c = [c, [c4, c5]];
    end

end


% function [c, ceq] = nonlcon(X, time, x1_0, x1_dot_0, y1_0, y1_dot_0, x2_0, x2_dot_0, y2_0, y2_dot_0, phi_0 , phi_dot_0, xo, yo, ro, L)
%     h = time(2)-time(1);
%     n = length(time);
%     x_list = [x1_0, x1_dot_0, y1_0, y1_dot_0, x2_0, x2_dot_0, y2_0, y2_dot_0, phi_0 , phi_dot_0];
%     u_list = [X(1), X(2), X(3), X(4)];
% 
%     n_vars = 14*(n-1) + 4;
% 
%     i = 5;
%     while i <= n_vars
%         x_list = [x_list; [X(i), X(i+1), X(i+2), X(i+3), X(i+4), X(i+5), X(i+6), X(i+7), X(i+8), X(i+9)]];
%         u_list = [u_list; [X(i+10), X(i+11), X(i+12), X(i+13)]];
%         i = i + 14;
%     end
% 
%     c = [];
%     ceq = [];
% 
%     X_1 = x_list(1,:)';
%     U_1 = u_list(1,:)';
%     X_1_dot = system_dynamics(X_1, U_1)';
% 
%     x1_temp = X_1(1);
%     x1_dot_temp = X_1(2);
%     y1_temp = X_1(3);
%     y1_dot_temp = X_1(4);
%     x2_temp = X_1(5);
%     x2_dot_temp = X_1(6);
%     y2_temp = X_1(7);
%     y2_dot_temp = X_1(8);
%     phi_temp = X_1(9);
%     phi_dot_temp = X_1(10);
% 
%     x1_ddot_temp = X_1_dot(2);
%     y1_ddot_temp = X_1_dot(4);
%     x2_ddot_temp = X_1_dot(6);
%     y2_ddot_temp = X_1_dot(8);
% 
%     c1 = (x1_temp-x2_temp)^2 + (y1_temp-y2_temp)^2 - L^2;
%     % c2 = (x1_dot_temp-x2_dot_temp)*cos(phi_temp) + (y1_dot_temp-y2_dot_temp)*sin(phi_temp);
%     % c3 = (x1_ddot_temp-x2_ddot_temp)*cos(phi_temp) + (y1_ddot_temp-y2_ddot_temp)*sin(phi_temp);
% 
%     ceq = [ceq, c1];
% 
%     c4 = ro^2 + 3 - (x1_temp-xo)^2 - (y1_temp-yo)^2;
%     c5 = ro^2 + 3 - (x2_temp-xo)^2 - (y2_temp-yo)^2;
% 
%     c = [c, [c4, c5]];
% 
%     for i = 1:n-1
%         xk = x_list(i,:)';
%         xk1 = x_list(i+1,:)';
%         uk = u_list(i,:)';
%         uk1 = u_list(i+1,:)';
% 
%         err = error_const(xk, xk1, uk, uk1, h)';
%         ceq = [ceq, err];
% 
%         X_1 = xk1;
%         U_1 = uk1;
%         X_1_dot = system_dynamics(X_1, U_1)';
% 
%         x1_temp = X_1(1);
%         x1_dot_temp = X_1(2);
%         y1_temp = X_1(3);
%         y1_dot_temp = X_1(4);
%         x2_temp = X_1(5);
%         x2_dot_temp = X_1(6);
%         y2_temp = X_1(7);
%         y2_dot_temp = X_1(8);
%         phi_temp = X_1(9);
%         phi_dot_temp = X_1(10);
% 
%         x1_ddot_temp = X_1_dot(2);
%         y1_ddot_temp = X_1_dot(4);
%         x2_ddot_temp = X_1_dot(6);
%         y2_ddot_temp = X_1_dot(8);
% 
%         c1 = (x1_temp-x2_temp)^2 + (y1_temp-y2_temp)^2 - L^2;
%         % c2 = (x1_dot_temp-x2_dot_temp)*cos(phi_temp) + (y1_dot_temp-y2_dot_temp)*sin(phi_temp);
%         % c3 = (x1_ddot_temp-x2_ddot_temp)*cos(phi_temp) + (y1_ddot_temp-y2_ddot_temp)*sin(phi_temp);
% 
%         ceq = [ceq, c1];
% 
%         c4 = ro^2 + 2 - (x1_temp-xo)^2 - (y1_temp-yo)^2;
%         c5 = ro^2 + 2 - (x2_temp-xo)^2 - (y2_temp-yo)^2;
% 
%         c = [c, [c4, c5]];
% 
%     end
% 
% end