function J = jacobianGeom(AA, jTyp)

N = size(AA, 3); % Número de juntas
if nargin < 2 % Se poucos argumentos forem fornecidos
    jTyp = 0;
end
if numel(jTyp) < 2 % Se não for um vetor
    jTyp = zeros(N, 1);
end

Zis = JointAxes(AA); % Os eixos das juntas
Org = LinkOrigins(AA); % As origens dos links
ON = Org(:, end); % TCP (último sistema)

% Inicializa as matrizes Jv e Jw
Jv = zeros(3, N);
Jw = zeros(3, N);

for i = 1:N % Para cada junta...
    Oi = Org(:, i); % ...obtém sua origem
    Zi = Zis(:, i); % ...obtém seu eixo z
    if i == 1 || i == 2
        Jvi = [0 0 0];
        Jwi = [0 0 0];
    elseif jTyp(i) == 0 % Se rotacional
        Jvi = cross(Zi, ON - Oi);
        Jwi = Zi;
    end
    
    Jv(:, i) = Jvi; % Define o componente Jv
    Jw(:, i) = Jwi; % Define o componente Jw
end

J = [Jv; Jw]; % Compõe o J final
end
