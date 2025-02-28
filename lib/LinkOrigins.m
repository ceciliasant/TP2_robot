function Org =  LinkOrigins(AA)
    % AA - hipermatriz of robot G.T.
    % Org - matrix 3 x (size(AA,3) + 1) with links' origins
    % this will calculate a matrix where the columns are the origin points
    % of A1, A2, ..., A_N

Org = zeros(3, size(AA,3)+1); % init Org

T = eye(4);
for n = 1: size(AA,3)
    T = T * AA(:,:,n);
    Org(:,n+1) = T(1:3,4);
end