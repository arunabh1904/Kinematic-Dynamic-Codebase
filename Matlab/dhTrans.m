function [ H ] = dhTrans( a, d, alpha, theta, n, m)
% dhTrans Returns the homogenous transform corresponding to the provide DH parameters for a set of links.
%


F = zeros(4, 4, 1, length(a));
n = n+1;
H = eye(4);

for i = 1:length(a)
    F(:,:,1,i) = translationz(d(i))*(([1 0 0; 0 1 0; 0 0 1; 0 0 0]*rotZ(theta(i))*[1 0 0 0; 0 1 0 0; 0 0 1 0])+[0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 1])*translationx(a(i))*(([1 0 0; 0 1 0; 0 0 1; 0 0 0]*rotX(alpha(i))*[1 0 0 0; 0 1 0 0; 0 0 1 0])+[0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 1]);
end

for i = n:m
    H = H*F(:,:,1,i);
end

end
