function [X,Y,Z]=RobotEndPatch(AAA)

X= zeros(1,size(AAA,4));
Y=X;
Z=X;
for n=1:size(AAA,4) % no positions
    A=eye(4); % identity matrix
    for j=1:size(AAA,3) % no links
        A=A*AAA(:,:,j,n);
    end
    X(n)=A(1,4);
    Y(n)=A(2,4);
    Z(n)=A(3,4);
end
