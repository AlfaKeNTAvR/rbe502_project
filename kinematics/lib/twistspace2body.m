function V_b = twistspace2body(V_s,T)
    % Rotation matrix and transition vector.
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    
    % Adjoint transformation matrix from Space to Body.
    Ad_Tbs = [ R'  zeros(3, 3); 
              -R' * skew(p) R'];
    
    % Twist in Body frame.
    V_b = Ad_Tbs * V_s; 
end