function q_adjusted = adjustAngles(q)
    % Adjust joint angles based on the given condition
    % If angle < pi, add 2*pi
    % If angle > pi, subtract 2*pi

    q_adjusted = q; % Initialize adjusted angles

    % Loop through each joint angle and apply the conditions
    for i = 3:length(q)
        if q(i) < -pi
            q_adjusted(i) = q(i) + 2*pi; % Add 2*pi if less than pi
        elseif q(i) > pi
            q_adjusted(i) = q(i) - 2*pi; % Subtract 2*pi if greater than pi
        end
    end
end