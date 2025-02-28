function AA=Tlinks(DH)
    % DH - matrix of kinem. parameters
    %       theta_1, l_1, d_1, alpha_1
    %       theta_2, l_2, d_2, alpha_2
    % AA - set of A G.T.
AA = zeros(4,4,size(DH,1)); % initialize AA

for n=1:size(DH,1)
    th = DH(n,1); l = DH(n,2); d = DH(n,3); alpha = DH(n,4);
    AA(:,:,n) = Tlink(th,l,d,alpha);
end