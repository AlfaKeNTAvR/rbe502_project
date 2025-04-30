function M = computeM(q)
    % computeM returns the inertia matrix M(q)
    % q is a 3x1 vector
    
    q2 = q(2); 
    q3 = q(3);
    
    M = [(9*cos(2*q2 + q3))/800 + (27*cos(2*q2))/800 + (9*cos(2*q2 + 2*q3))/3200 + (9*cos(q3))/800 + 117/3200, 0, 0;
        0, (9*cos(q3))/400 + 117/1600, (9*cos(q3))/800 + 9/1600;
        0, (9*cos(q3))/800 + 9/1600, 9/1600];
end