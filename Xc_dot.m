function x = Xc_dot(X_k, X_k1, u_k, u_k1, h)

    x = (-3)/(2*h)*(X_k-X_k1)-(1/4)*(system_dynamics(X_k, u_k) + system_dynamics(X_k1, u_k1));

end