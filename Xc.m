function x = Xc(X_k, X_k1, u_k, u_k1, h)

    x = 0.5*(X_k + X_k1) + h/8*(system_dynamics(X_k, u_k) - system_dynamics(X_k1, u_k1));

end