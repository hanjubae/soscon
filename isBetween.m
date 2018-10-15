function [result] = isBetween(value,A,B)
%Checks if a value lies inbetween A and B, regardless of order
%   Has +-0.0001 buffer

    if (((value >= A-0.0001) && (value <= B+0.0001)) || ((value >= B-0.0001) && (value <= A+0.0001)))
        result = 1;
    else
        result = 0;
    end
end

