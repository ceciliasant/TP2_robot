function [LeftHandle, RightHandle]= RenderRobot(DH_left, DH_right)
% NN- pontos no caminho
% DH
% jType- 0,1 rot/prism
%sScale- escala para o sist eixos


% [P,F] = seixos3(sScale); 

transformationMatrixLeft= Tlinks(DH_left);
OriginsLeft= LinkOrigins(transformationMatrixLeft);
LeftHandle=DrawLinks(OriginsLeft);
% LeftFrames=DrawFrames(transformationMatrixLeft,P,F);


transformationMatrixRight= Tlinks(DH_right);
OriginsRight= LinkOrigins(transformationMatrixRight);
RightHandle=DrawLinks(OriginsRight);
% RightFrames=DrawFrames(transformationMatrixRight,P,F);
end

