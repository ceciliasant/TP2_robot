%% Tranformações homogéneas
function htrans = htrans(x,y,z)
    htrans = [1 0 0 x
    0 1 0 y
    0 0 1 z
    0 0 0 1];
end