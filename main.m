clear;close all; clc;
addpath lib\
addpath lib\class_functions\

% === VALUES ===
filename = 'tp2.txt';
if exist(filename, 'file')
    % --- File exists
    [HBL, LBL, WBL, LD, LC, LB, LA, LX, H, LZ, HTA, STF, HTB, DTF, LTF, DTT, LTT, HTC, WTS]= readParams(filename);
else
    % --- Default values when file can't be found
    HBL = 50; LBL = 150; WBL = 40;  LD=194; LC=335;LB=340; LA=292; LX=125; H=1223; LZ=332; HTA=804; STF=219; HTB=704; DTF=433; LTF=1604; DTT=691; LTT=1104; HTC=804; WTS=600;
end
NN=100;
% === DH MATRIX ===
% --- Left arm
DH_left= [
    0, 0, H-LZ, 0; % J0
    0, LX, LZ, pi/2;  
    0, 0, LA, -pi/2;
    0, 0, 0, pi/2;
    0, 0, LB, -pi/2;
    0, 0, 0, pi/2;
    0, 0, LC, -pi/2;
    0, 0, 0, pi/2;
    0, 0 , LD, 0 % end
];

% --- Right arm
DH_right= [
    0, 0, H-LZ, 0;
    0, LX, LZ, pi/2;  
    0, 0, -LA, -pi/2;
    0, 0, 0, -pi/2;
    0, 0, LB, pi/2;
    0, 0, 0, -pi/2;
    0, 0, LC, pi/2;
    0, 0, 0, -pi/2;
    0, 0 , LD, 0
];

% === TABLES, BLOCKS, ROBOT, CLAWS ===
grid on;
axis equal;axis([-2500 2500 -1500 1500 0 1500]);
view(130, 30);
xlabel('X'); ylabel('Y'); zlabel('Z');

% --- Left Front Conveyor
createConveyor(LTF, HTA, WTS, [LTF + DTF, -((STF/2) + WTS), 0]);

% --- Right Front Conveyor
createConveyor(LTF, HTB, WTS, [LTF + DTF, STF/2, 0]);

% --- Back Conveyor
createConveyor(LTT, HTC, WTS, [-DTT, -WTS/2, 0]);

% --- Left Block
block1_start = hrotx(pi/2)*htrans(DTF + LTF-WBL/2, -(STF / 2 + WTS / 2 ), HTA);
block1= Block3D(LBL,WBL,HBL,block1_start,'r',0.7);

% --- Right Block
block2_start = hrotx(-pi/2)*htrans(DTF + LTF-WBL/2, +(STF / 2 + WTS / 2), HTB);
block2= Block3D(LBL,WBL,HBL,block2_start,'b',0.7);

% --- Render robot at zero hardware
[LeftHandle, RightHandle]= RenderRobot(DH_left,DH_right);

% --- Left claw
clawLstart= hrotx(pi/2)*htrans(LX, -(LA+LB+LC+LD), H);
ClawLeft= Claw3D(LBL/2,WBL+5,HBL/2,clawLstart,'#837F80');

% --- Right claw
clawRstart=hrotx(-pi/2) * htrans(LX, LA+LB+LC+LD, H)*hrotz(pi);
ClawRight= Claw3D(LBL/2,WBL+5,HBL/2,clawRstart,'#837F80');

% === 1. MOVE TO ABOVE BLOCKS (INVERSE) ===

% --- Target positions above the blocks
offset_z= HBL*2;

block_x_left = DTF+WBL/2;
block_y_left = -(STF / 2 + WTS / 2);
block_z_left = HTA;
block1_position= [block_x_left,block_y_left,block_z_left];

block_x_right = DTF+WBL/2;
block_y_right = STF / 2 + WTS / 2;
block_z_right = HTB;
block2_position= [block_x_right, block_y_right,block_z_right];

% --- Desired poses
T_des_left = htrans(block_x_left, block_y_left, block_z_left + offset_z) * hrotx(pi);
T_des_right = htrans(block_x_right, block_y_right, block_z_right + offset_z) * hrotx(pi);

% --- Initial joint guesses
q0_left = zeros(size(DH_left, 1), 1);
q0_right = zeros(size(DH_right, 1), 1);

% --- Inverse kinematics
tol = 1e-6;
max_iters = 100;
lambda = 0.1;
jTypes = zeros(1, size(DH_left, 1));

[q_left, ~, ~] = invKinematics(DH_left, q0_left, T_des_left, tol, max_iters, lambda, jTypes);
[q_right, ~, ~] = invKinematics(DH_right, q0_right, T_des_right, tol, max_iters, lambda, jTypes);

QQ_left=[q0_left q_left];
QQ_right=[q0_right q_right];


% --- From initial position and final for left arm
Qi_left=QQ_left(:, 1); 
Qf_left=QQ_left(:, 2);
MQ_left= LinspaceVect(Qi_left, Qf_left, NN);

MDH_left=GenerateMultiDH(DH_left, MQ_left, jTypes);

AAA_left = CalculateRobotMotion(MDH_left);
    
% --- From initial position and final for right arm
Qi_right=QQ_right(:, 1); 
Qf_right=QQ_right(:, 2);
MQ_right= LinspaceVect(Qi_right, Qf_right, NN);

MDH_right=GenerateMultiDH(DH_right, MQ_right, jTypes);

AAA_right = CalculateRobotMotion(MDH_right);
 
% === 2. MOVE DOWN TO BLOCKS (DIFERENTIAL) ===

% --- Start positions above the blocks
block1_start=[block_x_left,block_y_left,block_z_left + offset_z];
block2_start= [block_x_right, block_y_right,block_z_right + offset_z];

% --- Target on the blocks
block1_position= [block_x_left,block_y_left,block_z_left+HBL];
block2_position= [block_x_right, block_y_right,block_z_right+HBL];

% --- Create paths
P_left = [block1_position'-block1_start';0;0;0];
P_right = [block2_position'-block2_start';0;0;0];

% --- Changes
dr_left= P_left/NN;
dr_right= P_right/NN;

% --- Initial joint configurations
Q_left = q_left;
Q_right = q_right;

% --- Jacobian-based updates
for n = 1:NN-1
    % Left arm
    MDH_left = GenerateMultiDH(DH_left, Q_left, jTypes);
    AA_left = Tlinks(MDH_left(:,:,1));
    J_left = jacobianGeom(AA_left, jTypes);
    Ji_left = pinv(J_left); 
    dq_left = Ji_left * dr_left;

    Q_left = Q_left + dq_left; % Update joints
    leftQQ(:,n) = Q_left;

    % Right arm
    MDH_right = GenerateMultiDH(DH_right, Q_right, jTypes);
    AA_right = Tlinks(MDH_right(:,:,1));
    J_right = jacobianGeom(AA_right, jTypes);
    Ji_right = pinv(J_right); 
    dq_right = Ji_right * dr_right; % Joint increments
    Q_right = Q_right + dq_right; % Update joints
    rightQQ(:,n) = Q_right; % Store joint configuration
end


% --- From initial position and final for left arm
MDH_left=GenerateMultiDH(DH_left, leftQQ, jTypes);
newAAA_left = zeros(4,4,height(DH_left),size(AAA_left,4)+NN-1);
newAAA_left(:,:,:,1:end-NN+1) = AAA_left;
newAAA_left(:,:,:,end-NN+2:end) = CalculateRobotMotion(MDH_left);
AAA_left = newAAA_left;

% --- From initial position and final for right arm
MDH_right=GenerateMultiDH(DH_right, rightQQ, jTypes);
newAAA_right = zeros(4,4,height(DH_right),size(AAA_right,4)+NN-1);
newAAA_right(:,:,:,1:end-NN+1) = AAA_right;
newAAA_right(:,:,:,end-NN+2:end) = CalculateRobotMotion(MDH_right);
AAA_right = newAAA_right;

% === 3. MOVE UP TO NEAR THE JOINING POSITION (INVERSE) ===

% --- Target positions near the joining
offset_Y= LBL;
joining_pos_x= DTT+WBL/2;
joining_pos_y= 0;
joining_pos_z= H-LD;
% --- Desired poses
T_des_left =  htrans(joining_pos_x,joining_pos_y- offset_Y, joining_pos_z)*hrotx(pi);
T_des_right = htrans(joining_pos_x,joining_pos_y+ offset_Y, joining_pos_z)*hrotx(pi);

% --- Initial joint guesses
q0_left = Q_left;
q0_right = Q_right;

% --- Inverse kinematics

[q_left, iter_left, errors_left] = invKinematics(DH_left, q0_left, T_des_left, tol, max_iters, lambda, jTypes);
[q_right, iter_right, errors_right] = invKinematics(DH_right, q0_right, T_des_right, tol, max_iters, lambda, jTypes);

QQ_left=[q0_left q_left];
QQ_right=[q0_right q_right];


% --- From initial position and final for left arm
Qi_left=QQ_left(:, 1); 
Qf_left=QQ_left(:, 2);
MQ_left= LinspaceVect(Qi_left, Qf_left, NN);

MDH_left=GenerateMultiDH(DH_left, MQ_left, jTypes);
newAAA_left = zeros(4,4,height(DH_left),size(AAA_left,4)+NN);
newAAA_left(:,:,:,1:end-NN) = AAA_left;
newAAA_left(:,:,:,end-NN+1:end) = CalculateRobotMotion(MDH_left);
AAA_left = newAAA_left;

% --- From initial position and final for right arm
Qi_right=QQ_right(:, 1); 
Qf_right=QQ_right(:, 2);
MQ_right= LinspaceVect(Qi_right, Qf_right, NN);

MDH_right=GenerateMultiDH(DH_right, MQ_right, jTypes);
newAAA_right = zeros(4,4,height(DH_right),size(AAA_right,4)+NN);
newAAA_right(:,:,:,1:end-NN) = AAA_right;
newAAA_right(:,:,:,end-NN+1:end) = CalculateRobotMotion(MDH_right);
AAA_right = newAAA_right;
    
% === 4. JOIN BLOCKS (DIFERENTIAL) ===

% --- Start positions
block1_start=[joining_pos_x,joining_pos_y-offset_Y, joining_pos_z];
block2_start= [joining_pos_x,joining_pos_y+offset_Y, joining_pos_z];

% --- Target 
block1_position= [joining_pos_x,joining_pos_y-LBL/2, joining_pos_z];
block2_position= [joining_pos_x,joining_pos_y+LBL/2, joining_pos_z];

% --- Create linear paths
P_left = [block1_position'-block1_start';0;0;0];
P_right = [block2_position'-block2_start';0;0;0];

% --- Differential changes
dr_left= P_left/NN;
dr_right= P_right/NN;

% --- Initial joint configurations
Q_left = q_left;
Q_right = q_right;

% --- Jacobian
for n = 1:NN-1
    % Left arm
    MDH_left = GenerateMultiDH(DH_left, Q_left, jTypes);
    AA_left = Tlinks(MDH_left(:,:,1));
    J_left = jacobianGeom(AA_left, jTypes);
    Ji_left = pinv(J_left); 
    dq_left = Ji_left * dr_left;
    Q_left = Q_left + dq_left; % Update joints
    leftQQ(:,n) = Q_left;

    % Right arm
    MDH_right = GenerateMultiDH(DH_right, Q_right, jTypes);
    AA_right = Tlinks(MDH_right(:,:,1));
    J_right = jacobianGeom(AA_right, jTypes);
    Ji_right = pinv(J_right);
    dq_right = Ji_right * dr_right; % Joint increments
    Q_right = Q_right + dq_right; % Update joints
    rightQQ(:,n) = Q_right; % Store joint configuration
end


% --- From initial position and final for left arm
MDH_left=GenerateMultiDH(DH_left, leftQQ, jTypes);
newAAA_left = zeros(4,4,height(DH_left),size(AAA_left,4)+NN-1);
newAAA_left(:,:,:,1:end-NN+1) = AAA_left;
newAAA_left(:,:,:,end-NN+2:end) = CalculateRobotMotion(MDH_left);
AAA_left = newAAA_left;

% --- From initial position and final for right arm
MDH_right=GenerateMultiDH(DH_right, rightQQ, jTypes);
newAAA_right = zeros(4,4,height(DH_right),size(AAA_right,4)+NN-1);
newAAA_right(:,:,:,1:end-NN+1) = AAA_right;
newAAA_right(:,:,:,end-NN+2:end) = CalculateRobotMotion(MDH_right);
AAA_right = newAAA_right;

% === 5. ROTATE AT J0 (DIRECT) ===

% --- Start and end positions
block1_start = [joining_pos_x, joining_pos_y - LBL / 2, joining_pos_z];
block2_start = [joining_pos_x, joining_pos_y + LBL / 2, joining_pos_z];

block1_position = [-joining_pos_x, joining_pos_y - LBL / 2, joining_pos_z];
block2_position = [-joining_pos_x, joining_pos_y + LBL / 2, joining_pos_z];

theta0_rotation = pi; % Rotation around Z-axis by pi (joint 0)

x_left_path = linspace(block1_start(1), block1_position(1), NN);
y_left_path = linspace(block1_start(2), block1_position(2), NN);
z_left_path = linspace(block1_start(3), block1_position(3), NN);

leftQQ = zeros(length(Q_left), NN);
AAA_right_new = zeros(4, 4, height(DH_right), NN);
AAA_left_new = zeros(4, 4, height(DH_left), NN);

for n = 1:NN
    % Interpolate rotation of joint 0 over N steps
    theta_step = theta0_rotation * (n / NN); % Intermediate rotation for joint 0

    % --- Update Joint 0 in Left and Right Joint Configurations ---
    Q_left(1) = theta_step;  % Update joint 0 for left arm
    Q_right(1) = theta_step; % Update joint 0 for right arm

    % --- Update Transformation Matrices ---
    MDH_left = GenerateMultiDH(DH_left, Q_left, jTypes);
    MDH_right = GenerateMultiDH(DH_right, Q_right, jTypes);

    % --- Update 
    AA_left_step = CalculateRobotMotion(MDH_left);
    AA_right_step = CalculateRobotMotion(MDH_right);
    leftQQ(:, n) = Q_left;
    AAA_left_new(:,:,:,n) = AA_left_step;
    AAA_right_new(:,:,:,n) = AA_right_step;
end

% --- Overall Trajectories ---
newAAA_left = cat(4, AAA_left, AAA_left_new);
newAAA_right = cat(4, AAA_right, AAA_right_new);

AAA_left = newAAA_left;
AAA_right = newAAA_right;

% === 6. PUT DOWN THE JOINED BLOCKS (DIFERENTIAL) ===

% --- Start positions
block1_start= [-joining_pos_x, joining_pos_y - LBL / 2, joining_pos_z];
block2_start= [-joining_pos_x, joining_pos_y + LBL / 2, joining_pos_z];

% --- Target 
block1_position= [-joining_pos_x,joining_pos_y-LBL/2, HTC+HBL];
block2_position= [-joining_pos_x,joining_pos_y+LBL/2, HTC+HBL];

% --- Create paths

P_left = [block1_position'-block1_start';0;0;0];
P_right = [block2_position'-block2_start';0;0;0];


% --- Differential changes
dr_left= P_left/NN;
dr_right= P_right/NN;

% --- Initial joint configurations
Q_left = Q_left;
Q_right = Q_right;

for n = 1:NN-1
    % Left arm
    MDH_left = GenerateMultiDH(DH_left, Q_left, jTypes);
    AA_left = Tlinks(MDH_left(:,:,1));
    J_left = jacobianGeom(AA_left, jTypes);
    Ji_left = pinv(J_left); 
    dq_left = Ji_left * dr_left;

    Q_left = Q_left + dq_left; % Update joints
    leftQQ(:,n) = Q_left;

    % Right arm
    MDH_right = GenerateMultiDH(DH_right, Q_right, jTypes);
    AA_right = Tlinks(MDH_right(:,:,1));
    J_right = jacobianGeom(AA_right, jTypes);
    Ji_right = pinv(J_right); 
    dq_right = Ji_right * dr_right; % Joint increments
    Q_right = Q_right + dq_right; % Update joints
    rightQQ(:,n) = Q_right; % Store joint configuration
end

MDH_left=GenerateMultiDH(DH_left, leftQQ, jTypes);
newAAA_left = zeros(4,4,height(DH_left),size(AAA_left,4)+NN-1);
newAAA_left(:,:,:,1:end-NN+1) = AAA_left;
newAAA_left(:,:,:,end-NN+1:end) = CalculateRobotMotion(MDH_left);
AAA_left = newAAA_left;

% --- From initial position and final for right arm
MDH_right=GenerateMultiDH(DH_right, rightQQ, jTypes);
newAAA_right = zeros(4,4,height(DH_right),size(AAA_right,4)+NN-1);
newAAA_right(:,:,:,1:end-NN+1) = AAA_right;
newAAA_right(:,:,:,end-NN+2:end) = CalculateRobotMotion(MDH_right);
AAA_right = newAAA_right;

% === MOVE TO HARDWARE ZERO ===

% --- Start and End Joint Positions ---

% --- Target joint (hardware zero = all joints at 0)
Q_zero_left = zeros(length(Q_left), 1); 
Q_zero_right = zeros(length(Q_right), 1);

leftQQ = zeros(length(Q_left), NN); 
rightQQ = zeros(length(Q_right), NN);

AAA_left_new = zeros(4, 4, height(DH_left), NN); % New transformations (Left)
AAA_right_new = zeros(4, 4, height(DH_right), NN); % New transformations (Right)

% --- Interpolate Each Joint Configuration for Left and Right Arms
for n = 1:NN
    Q_left_step = Q_left + (Q_zero_left - Q_left) * (n / NN);
    Q_right_step = Q_right + (Q_zero_right - Q_right) * (n / NN);
    
    leftQQ(:, n) = Q_left_step;
    rightQQ(:, n) = Q_right_step;
    
    MDH_left_step = GenerateMultiDH(DH_left, Q_left_step, jTypes);
    MDH_right_step = GenerateMultiDH(DH_right, Q_right_step, jTypes);
    
    AAA_left_new(:, :, :, n) = CalculateRobotMotion(MDH_left_step);
    AAA_right_new(:, :, :, n) = CalculateRobotMotion(MDH_right_step);
end

newAAA_left = cat(4, AAA_left, AAA_left_new);
newAAA_right = cat(4, AAA_right, AAA_right_new);

AAA_left = newAAA_left;
AAA_right = newAAA_right;

% [X_left, Y_left, Z_left] = RobotEndPatch(AAA_left_new);
% % plot3(X_left, Y_left, Z_left, '.r');
% 
% [X_right, Y_right, Z_right] = RobotEndPatch(AAA_right_new);
% % plot3(X_right, Y_right, Z_right, '.b');

% === TRAJECTORIES ===

[X,Y,Z]=RobotEndPatch(AAA_right);
plot3(X,Y,Z, '.b', 'MarkerSize', 2);
hold on;
[X,Y,Z]=RobotEndPatch(AAA_left);
plot3(X,Y,Z, '.r', 'MarkerSize', 2);
pause(5);
% === ANIMATION ===

sd=0.01;

start_tables1= [DTF + LTF-WBL/2, -(STF / 2 + WTS / 2 ), HTA];
tables_T_block1= [DTF+WBL/2,-(STF / 2 + WTS / 2),HTA];
back_table_start1= [-DTT-WBL/2, LBL/2,HTC];
end_T_block1= [-DTT-LTT+WBL/2, LBL/2,HTC];

start_tables2= [DTF + LTF-WBL/2, STF / 2 + WTS / 2, HTB];
tables_T_block2= [DTF+WBL/2,STF / 2 + WTS / 2,HTB];
back_table_start2= [-DTT-WBL/2, -LBL/2,HTC];
end_T_block2= [-DTT-LTT+WBL/2, -LBL/2,HTC];

AnimateRobot(LeftHandle,RightHandle, AAA_left,AAA_right, ClawLeft, ClawRight,block1, block2,start_tables1,tables_T_block1,back_table_start1, end_T_block1,start_tables2,tables_T_block2, back_table_start2,end_T_block2,sd)