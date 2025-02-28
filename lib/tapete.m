function [P, F] = tapete(length, height, width)
    % Define the vertices of the table top
    P_top = [
        0, 0, height;            % Point 1: Top-left-front
        width, 0, height;        % Point 2: Top-right-front
        width, length, height;   % Point 3: Top-right-back
        0, length, height;       % Point 4: Top-left-back
        0, 0, height - height/5; % Point 5: Bottom-left-front
        width, 0, height - height/5; % Point 6: Bottom-right-front
        width, length, height - height/5; % Point 7: Bottom-right-back
        0, length, height - height/5      % Point 8: Bottom-left-back
    ];
    
    F_top = [
        1, 2, 3, 4;   % Top face
        5, 6, 7, 8;   % Bottom face
        1, 5, 6, 2;   % Front face
        2, 6, 7, 3;   % Right face
        3, 7, 8, 4;   % Back face
        4, 8, 5, 1    % Left face
    ];

    % Define the vertices of one leg
    leg_height = 4 * height / 5;
    leg_width = width / 10;
    leg_length = length / 10;
    
    P_leg = [
        0, 0, 0;                   % Point 9: Bottom-left-front
        leg_width, 0, 0;           % Point 10: Bottom-right-front
        leg_width, leg_length, 0;  % Point 11: Bottom-right-back
        0, leg_length, 0;          % Point 12: Bottom-left-back
        0, 0, leg_height;          % Point 13: Top-left-front
        leg_width, 0, leg_height;  % Point 14: Top-right-front
        leg_width, leg_length, leg_height; % Point 15: Top-right-back
        0, leg_length, leg_height; % Point 16: Top-left-back
    ];
    
    F_leg = [
        9, 10, 14, 13;  % Front face
        10, 11, 15, 14; % Right face
        11, 12, 16, 15; % Back face
        12, 9, 13, 16;  % Left face
        13, 14, 15, 16; % Top face
        9, 10, 11, 12;  % Bottom face
    ];

    % Apply translation to create the four legs
    P_leg_1 = P_leg;
    P_leg_2 = translate(P_leg, width - leg_width, 0, 0);
    P_leg_3 = translate(P_leg, 0, length - leg_length, 0);
    P_leg_4 = translate(P_leg, width - leg_width, length - leg_length, 0);

    % Combine all vertices
    P = [P_top; P_leg_1; P_leg_2; P_leg_3; P_leg_4];
    
    % Offset for faces of the legs
    offset = size(P_top, 1);
    F_leg_2 = F_leg + offset;
    F_leg_3 = F_leg + 2 * offset;
    F_leg_4 = F_leg + 3 * offset;

    % Combine all faces
    F = [F_top; F_leg; F_leg_2; F_leg_3; F_leg_4];

    % Apply rotation around Z axis by 90 degrees 
    Rz = hrotz(pi/2); % Rotation matrix for 90 degrees around Z axis 
    P = (Rz * [P'; ones(1, size(P, 1))])'; 
    P(:, 4) = []; % Remove the homogeneous coordinate

end


% function [P,F]= tapete(length, height, width)
%     % Define the vertices of the table top
%     P = [
%         0, 0, height;          % Point 1: Top-left-front
%         width, 0, height;     % Point 2: Top-right-front
%         width, length, height; % Point 3: Top-right-back
%         0, length, height;      % Point 4: Top-left-back
%         0, 0, height-height/5;
%         width, 0, height-height/5;     % Point 6: bottom-right-front
%         width, length, height-height/5; % Point 7: bottom-right-back
%         0, length, height-height/5      % Point 8: bottom-left-back
%     ];
% 
%     F = [
%         1, 2, 3, 4;  % Top face
%         5,6,7,8;
%         1,5,6,2;
%         2,6,7,3;
%         3,7,8,4;
%         4,8,5,1
%     ];
% 
%     P_leg= [0,0,0; % 9
%          width/10,0,0; %10
%          width/10,length/10,0; %11
%          0, length/10,0; %12
%          0,0,4*height/5; %13
%         width/10, 0, 4*height/5;     % Point 14: bottom-right-front
%         width/10, length/10, 4*height/5; % Point 15: bottom-right-back
%         0, length/10, 4*height/5      % Point 16: bottom-left-back
%     ];
%     F_leg=[9,10,11,12;
%           13,14,15,16;
%           9,10,14,13;
%           10,11,15,14;
%           12,11,15,16;
%           9,12,16,13];
%     P_leg_front_left= htrans(width-width/10,0,0)*P_leg;
% 
%     P= [P;P_leg;P_leg_front_left];
%     F=[F;F_leg];
% end
% function [P, F] = tapete(length, height, width)
%     % Define the vertices of the table top
%     P_top = [
%         0, 0, height;            % Point 1: Top-left-front
%         width, 0, height;        % Point 2: Top-right-front
%         width, length, height;   % Point 3: Top-right-back
%         0, length, height;       % Point 4: Top-left-back
%         0, 0, height - height/5; % Point 5: Bottom-left-front
%         width, 0, height - height/5; % Point 6: Bottom-right-front
%         width, length, height - height/5; % Point 7: Bottom-right-back
%         0, length, height - height/5      % Point 8: Bottom-left-back
%     ];
% 
%     F_top = [
%         1, 2, 3, 4;   % Top face
%         5, 6, 7, 8;   % Bottom face
%         1, 5, 6, 2;   % Front face
%         2, 6, 7, 3;   % Right face
%         3, 7, 8, 4;   % Back face
%         4, 8, 5, 1    % Left face
%     ];
% 
%     % Define the vertices of one leg
%     leg_height = 4 * height / 5;
%     leg_width = width / 10;
%     leg_length = length / 10;
% 
%     P_leg = [
%         0, 0, 0;                   % Point 9: Bottom-left-front
%         leg_width, 0, 0;           % Point 10: Bottom-right-front
%         leg_width, leg_length, 0;  % Point 11: Bottom-right-back
%         0, leg_length, 0;          % Point 12: Bottom-left-back
%         0, 0, leg_height;          % Point 13: Top-left-front
%         leg_width, 0, leg_height;  % Point 14: Top-right-front
%         leg_width, leg_length, leg_height; % Point 15: Top-right-back
%         0, leg_length, leg_height; % Point 16: Top-left-back
%     ];
% 
%     F_leg = [
%         9, 10, 14, 13;  % Front face
%         10, 11, 15, 14; % Right face
%         11, 12, 16, 15; % Back face
%         12, 9, 13, 16;  % Left face
%         13, 14, 15, 16; % Top face
%         9, 10, 11, 12;  % Bottom face
%     ];
% 
%     % Apply translation to create the four legs
%     P_leg_1 = P_leg;
%     P_leg_2 = translate(P_leg, width - leg_width, 0, 0);
%     P_leg_3 = translate(P_leg, 0, length - leg_length, 0);
%     P_leg_4 = translate(P_leg, width - leg_width, length - leg_length, 0);
% 
%     % Combine all vertices
%     P = [P_top; P_leg_1; P_leg_2; P_leg_3; P_leg_4];
% 
%     % Offset for faces of the legs
%     offset = size(P_top, 1);
%     F_leg_2 = F_leg + offset;
%     F_leg_3 = F_leg + 2 * offset;
%     F_leg_4 = F_leg + 3 * offset;
% 
%     % Combine all faces
%     F = [F_top; F_leg; F_leg_2; F_leg_3; F_leg_4];
% 
%     % Visualize the table
%     patch('Vertices', P, 'Faces', F, 'FaceColor', 'cyan', 'EdgeColor', 'black');
%     axis equal;
%     xlabel('X');
%     ylabel('Y');
%     zlabel('Z');
%     title('3D Table Geometry');
% end
