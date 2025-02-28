function Z=JointAxes(AA)
% devolve os vetores unitários dos eixos das N juntas
%  Z : matriz com 3 linhas por N+1 colunas
%  AA: hipermatriz (4x4xN) das transformaçoes geometricas, de Tlinks(DH)

Z= [0 0 1]';
N= size(AA, 3);
A = eye(4);
for n=1:N
    A=A* AA(:,:,n); % multiplicação cumulativa
    Z(:,n+1)= A(1:3,3); % vetor do eixo z atual
end
