function MQ = LinspaceVect(Qi, Qf, N)
% A funçao deve emular a operaçao de linspace sobre vetores.
% Qi- vetor dos valores iniciais
% Qf- vetor dos valores finais
% N- numero de elementos dos linspace
% MQ- matriz com todos os vetores — cada linha sera o linspace dos
% elementos correspondentes de Qi ate Qf.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    MQ = zeros(numel(Qi),N); 
    % número de elementos em Qi -- quantidade de linhas da matriz MQ
    % um linspace para cada elemento de Qi
    for i=1:numel(Qi)
        % sequência de N valores linearmente espaçados entre Qi(i) e Qf(i)
        MQ(i, :) = linspace(Qi(i), Qf(i), N);
        % linha i da matriz MQ com o resultado do linspace.
    end
end
