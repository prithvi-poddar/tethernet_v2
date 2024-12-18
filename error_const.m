function e = error_const(X_k, X_k1, u_k, u_k1, h)
    
    u_c = (u_k + u_k1)/2;
    e = Xc_dot(X_k, X_k1, u_k, u_k1, h) - system_dynamics(Xc(X_k, X_k1, u_k, u_k1, h), u_c);

end