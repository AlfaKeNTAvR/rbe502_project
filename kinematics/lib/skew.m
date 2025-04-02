function SSM = skew(omega)
    % SKEW Computes the skew-symmetric matrix of a 3-element vector.
    %   This function takes a 3-element vector and returns its corresponding
    %   skew-symmetric matrix. 
    
    %   Syntax:
    %       SSM = skew(omega)
    
    %   Input Argument:
    %       omega - A 3-element vector for which the skew-symmetric matrix
    %               is to be computed.
    
    %   Output Argument:
    %       SSM - The 3x3 skew-symmetric matrix of the input vector.
    
    %   Example:
    %       omega = [1; 2; 3];
    %       SSM = skew(omega)
    %       SSM = [0    -3     2;
    %              3     0    -1;
    %             -2     1     0]

    % Calculate omega skew-symmetric matrix.
    SSM = [0, -omega(3), omega(2); omega(3), 0, -omega(1); -omega(2), omega(1), 0];
end