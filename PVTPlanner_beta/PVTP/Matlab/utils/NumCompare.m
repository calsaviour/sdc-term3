function [ c ] = NumCompare( a, b, comp, margins )
%NumCompare Compare two numbers within the specified accuracy
%   Compare two numbers within the specified accuracy

    % Get specified epsilon
    epsilon = getMargin(margins, 'accuracy');

    % Calculate number difference
    if a > b
        diff = a - b;
    else
        diff = b - a;
    end
    
    areEqual = diff <= epsilon;
    
    % Compare
    switch lower(comp)
        case 'eq'
            c = areEqual;
        case 'lt'
            c = ~areEqual && (a < b);
        case 'le'
            c = areEqual || (a < b);
        case 'gt'
            c = ~areEqual && (a > b);
        case 'ge'
            c = areEqual || (a > b);
        case 'ne'
            c = ~areEqual;
    end
    
end

