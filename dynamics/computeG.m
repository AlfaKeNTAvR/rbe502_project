function G = computeG(q)
    % computeG returns the gravity torque vector G(q)
    % q = [q1; q2; q3]
    
    q2 = q(2);
    q3 = q(3);
    
    G = [ 0;
          (2943*cos(q2 + q3))/8000 + (8829*cos(q2))/4000;
          (2943*cos(q2 + q3))/8000 ];
    end