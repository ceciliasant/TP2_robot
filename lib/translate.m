function P_translated = translate(P, dx, dy, dz)
    % Apply translation to points P
    P_translated = P + repmat([dx, dy, dz], size(P, 1), 1);
end