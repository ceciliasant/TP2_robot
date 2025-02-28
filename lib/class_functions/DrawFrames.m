function H = DrawFrames(AA,P,F)
% AA - GT of robot
% P - Object Points
% F - Object Faces

H= cell(size(AA,3));
patch('Vertices', P(1:3,:)', 'Faces', F, 'FaceColor', 'w'); % base
% H = zeros(1, size(AA,3));

T = eye(4);
for n= 1 : size(AA,3)
    T = T * AA(:,:,n);
    Q= T*P;
    H{n}=patch('Vertices', Q(1:3,:)', 'Faces', F, 'FaceColor', rand(1,3));
end