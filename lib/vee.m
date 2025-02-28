function e=vee(T)
 % Auxiliary function for the vee() operation
 % Normally only used for 3x3 matrices but we abuse the concept
 % and extend it to larger matrices but use only the 3x3 submatrix
 if length(T) < 3 || width(T) < 3
 e=NaN;
 return
 end
 e=zeros(3,1); % column vector
 e(1)=T(3,2);
 e(2)=T(1,3);
 e(3)=T(2,1);