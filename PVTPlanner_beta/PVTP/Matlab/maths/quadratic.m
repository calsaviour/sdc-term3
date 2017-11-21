function [ b_plus, b_minus ] = quadratic( a, b, c, margins )
%quadratic Compute quadratic equation
%   Compute quadratic equation

    if NumCompare(a, 0, 'eq', margins)
        b_plus = -c / b;
        b_minus = b_plus;
        return;
    end
    
    % Try to prevent any numerical issues here
    discriminant = b^2 - 4 * a * c;
    
    if discriminant < 0
        discriminant = clipToZero(discriminant, margins);
    end
     
    if b < 0
        term = -0.5 * (b - sqrt(discriminant));
        b_plus =  term / a;
        b_minus = c / term;
    else
        term = -0.5 * (b + sqrt(discriminant));
        b_minus = term / a;
        b_plus = c / term;
    end

end

