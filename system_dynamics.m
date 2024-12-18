function dxdt = system_dynamics(X, u)

    % Constants
    L = 22;
    m1 = 2.5;
    m2 = 2.5;
    % object radius: 5.5

    % Control inputs
    F1 = u(1);
    F2 = u(2);
    theta1 = u(3);
    theta2 = u(4);

    % State variables
    x_com = X(1); 
    x_com_dot = X(2);
    y_com = X(3); 
    y_com_dot = X(4);
    phi = X(5); 
    
    if phi > 2*pi
        phi = phi - 2*pi;
    elseif phi < 0
        phi = 2*pi + phi;
    end
    phi_dot = X(6);
    z = X(7);

    F1x = F1*cos(theta1-phi);
    F1y = -F1*sin(theta1-phi);
    F2x = F2*cos(theta2-phi);
    F2y = -F2*sin(theta2-phi);

    % Accelerations
    x_com_ddot = (F1x + F2x)/(m1 + m2);
    y_com_ddot = (F1y + F2y)/(m1 + m2);
    phi_ddot = 2/(m1*L)*((F2y-F1y)*cos(phi) - (F2x-F1x)*sin(phi));
    z_dot = 0;

    % Pack derivatives into dzdt
    dxdt = z*[x_com_dot; x_com_ddot; y_com_dot; y_com_ddot; phi_dot; phi_ddot; z_dot];
end