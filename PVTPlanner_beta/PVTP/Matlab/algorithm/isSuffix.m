function [ b ] = isSuffix( B, B_prime )
%isSuffix Whether B is a suffix of B_prime
%   Whether B is a suffix of B_prime

    b = 0;

    s_B = size(B);
    s_B_prime = size(B_prime);
    
    % If B is longer than B_prime, it can't be a suffix
    if s_B(1, 2) > s_B_prime(1, 2)
        return;
    end
    
    % Otherwise, check that B is the suffix
    j = s_B_prime(1, 2);
    for i=s_B(1, 2):-1:1
        
        % Compare last bit
        if B(1, i) ~= B_prime(1, j)
            return;
        end
        
        % Move up
        j = j - 1;
        
    end
    
    % If we make it here, B is a suffix of B_prime
    b = 1;

end

