function MDH = GenerateMultiDH(DH, MQ, t)
    % Function to generate a hypermatrix of DH matrices for various joint configurations.
    %
    % Inputs:
    % DH - Base DH matrix (for home position)
    % MQ - Matrix of joint configurations where each column represents a set of joint angles
    % t  - (optional) Vector indicating joint types: 0 for rotational, 1 for prismatic
    %
    % Outputs:
    % MDH - 3D hypermatrix of DH matrices for each joint configuration
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if nargin < 3
        % If t is not provided, assume all joints are rotational
        t = zeros(1, size(DH, 1));
    end

    N = size(MQ, 2);  % Number of columns in MQ (each column is a set of joint angles)  
    % Get the size of the DH matrix (4x4)
    [rows, cols] = size(DH);
    % Initialize the 3D hypermatrix MDH with zeros
    MDH = zeros(rows, cols, N);
    
    % Loop through each joint configuration (each column in MQ)
    for i = 1:N
        % For each column in MQ, replace the variables in DH with the joint values from MQ
        % Assuming DH has symbolic variables that need to be substituted
        MDH(:, :, i) = DH;
        for k = 1:length(t)
            if t(k) == 0  % Rotational joint
                MDH(k, 1, i) = MDH(k, 1, i)+ MQ(k, i);
            elseif t(k) == 1  % Prismatic joint
                MDH(k, 3, i) = MQ(k, i);
            end
        end
    end
end