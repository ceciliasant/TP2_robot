function AnimateRobot(LeftHandle,RightHandle, AAA_left,AAA_right, claw1, claw2,block1, block2,start_tables1,tables_T_block1,back_table_start1, end_T_block1,start_tables2,tables_T_block2, back_table_start2,end_T_block2,sd)
% AnimateRobot - Anima o movimento de um braço robótico com base nas 
% matrizes de transformação.
%
% Parâmetros de Entrada:
% 
% AAA - superhipermatriz que contém a sequência temporal de hipermatrizes calculadas por CalculateRobotMotion;
% claw- object claw
% block- object block



for n=1:size(AAA_left,4) % iterar pelas posições
    AA_left=AAA_left(:,:,:,n); % tranformações de todos os elos em cada posição
    Org_left= LinkOrigins(AA_left); % origens dos elos

    AA_right=AAA_right(:,:,:,n); % tranformações de todos os elos em cada posição
    Org_right= LinkOrigins(AA_right); % origens dos elos

    % ajustar a nova posição dos elos
    LeftHandle.XData= Org_left(1,:); % todos os x da linha (elos robot)
    LeftHandle.YData= Org_left(2,:); % todos os y da linha (elos robot)
    LeftHandle.ZData= Org_left(3,:); % todos os z da linha (elos robot)

    RightHandle.XData= Org_right(1,:); % todos os x da linha (elos robot)
    RightHandle.YData= Org_right(2,:); % todos os y da linha (elos robot)
    RightHandle.ZData= Org_right(3,:); % todos os z da linha (elos robot)

    % calcular a transformaco geométrica acumulada
    T_left= eye(4); % identidade
    T_right= eye(4);

    for j= 1:size(AAA_left,3) % para atualizar cada sist eixos
        T_left=T_left*AAA_left(:,:,j,n); %transformaco geométrica acumulada
        T_right=T_right*AAA_right(:,:,j,n);
    end
    
    claw1= claw1.updatePosition(T_left); % claw moves with arm
    claw2= claw2.updatePosition(T_right);
    
    % animate blocks on the front tables
    if and(n>=0,n<=100)

        % Block 1 interpolation: tables_T_block1 -> end_T_block1
        x1_path = linspace(start_tables1(1), tables_T_block1(1), 101);
        y1_path = linspace(start_tables1(2), tables_T_block1(2),101);
        z1_path = linspace(start_tables1(3),tables_T_block1(3),101);
        
        % Block 2 interpolation: tables_T_block2 -> end_T_block2
        x2_path = linspace(start_tables2(1),tables_T_block2(1), 101);
        y2_path = linspace(start_tables2(2),tables_T_block2(2), 101);
        z2_path = linspace(start_tables2(3),tables_T_block2(3), 101);

        index = n + 1; % Interpolation index
        x1 = x1_path(index); y1 = y1_path(index); z1 = z1_path(index);
        block1 = block1.updatePosition(htrans(x1, y1, z1));

        x2 = x2_path(index); y2 = y2_path(index); z2 = z2_path(index);
        block2 = block2.updatePosition(htrans(x2, y2, z2));
        
    elseif n >= 199 && n <= 597  % Phase 2: Block moves with the arm
        block1 = block1.updatePosition(T_left); % Block follows robot end-effector
        block2 = block2.updatePosition(T_right); % Block follows robot end-effector
     
    elseif n > 597  % Phase 3: Block at final destination on table
        
        % Block 1 interpolation: back_table_start1 -> end_T_block1
        x1_path = linspace(back_table_start1(1), end_T_block1(1), 101);
        y1_path = linspace(back_table_start1(2), end_T_block1(2),101);
        z1_path = linspace(back_table_start1(3),end_T_block1(3),101);
        
        % Block 2 interpolation: back_table_start2 -> end_T_block2
        x2_path = linspace(back_table_start2(1),end_T_block2(1), 101);
        y2_path = linspace(back_table_start2(2),end_T_block2(2), 101);
        z2_path = linspace(back_table_start2(3),end_T_block2(3), 101);

        index = n - 597 + 1; % Interpolation index
        x1 = x1_path(index); y1 = y1_path(index); z1 = z1_path(index);
        block1 = block1.updatePosition(htrans(x1, y1, z1));

        x2 = x2_path(index); y2 = y2_path(index); z2 = z2_path(index);
        block2 = block2.updatePosition(htrans(x2, y2, z2));
      
    end
    
    pause(sd); % Small delay for animation
end
end