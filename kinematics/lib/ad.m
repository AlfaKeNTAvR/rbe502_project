function adV = ad(V)
    % Angular and Linear components of a Twist V.
    omega = V(1:3);
    v = V(4:6);
    
    % Lie bracket of V.
    adV = [skew(omega) zeros(3); skew(v) skew(omega)]; 
end