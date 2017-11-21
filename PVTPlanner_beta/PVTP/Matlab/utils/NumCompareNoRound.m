function [ c ] = NumCompareNoRound( a, b, comp, margins )
%NumCompare Compare two numbers within the specified accuracy
%   Compare two numbers within the specified accuracy

    % Get specified accuracy
    accuracy = getMargin(margins, 'accuracy');

    % Truncate numbers
    a = truncdec(a, accuracy, 0);
    b = truncdec(b, accuracy, 0);
    
    % Compare
    switch lower(comp)
        case 'eq'
            c = a == b;
        case 'lt'
            c = a < b;
        case 'le'
            c = a <= b;
        case 'gt'
            c = a > b;
        case 'ge'
            c = a >= b;
        case 'ne'
            c = a ~= b;
    end
    
end