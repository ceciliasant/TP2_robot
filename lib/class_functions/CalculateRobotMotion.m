function AAA=CalculateRobotMotion(MDH)
% MDH - hipermatrix 4x4x numjuntas
% from GenerateMultiDH
% AAA - hipermatriz; robot positions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
AAA= zeros(4,4,size(MDH,1),size(MDH,3));

for n = 1:size(MDH,3) % iteração no tempo
    AAA(:,:,:,n)= Tlinks(MDH(:,:,n)); % uma folha por cada junta
end