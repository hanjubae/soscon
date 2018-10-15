function [output_angle, new_priorValues] = steer(input_angle,priorValues, last_steer)
%PI Controller for the robot's steering angle
%   priorValues is a variable kept in the calling scope
%   priorValues is a column vector of the past 5 input headings

    sz = size( priorValues, 1 );
    new_priorValues = circshift(priorValues,-1);
    new_priorValues(sz,1) = input_angle;
    
    for i = 1:sz
        temp = priorValues(i) - last_steer;
        if (temp > 180)
            
        elseif (temp < 180)
            
        end
    end
    
    %Adjusts these coefficients to tune PI performance
    output_angle = 0.9*input_angle + 0.1*(mean(priorValues));
end

