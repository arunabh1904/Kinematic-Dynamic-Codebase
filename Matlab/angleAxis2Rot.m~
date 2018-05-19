%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Function name: angleAxis2Rot
%Returns the rotation matrix, given a rotation axis k, rotation angle theta

%[R] = angleAxis2Rot(k,theta)

%R = the rotation matrix describing rotation about k axis for a given theta

%k = the vector axis about which the coordinate frame rotation is performed
%theta = angle with which the rotation is performed in radians

%Name: Vineet Pandey
%CWID: 10826588
%Course Number: MEGN544
%Date: 09/29/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function R = angleAxis2Rot(k, theta)
Sk=cpMap(k);
R=eye(3)*cos(theta)+Sk*sin(theta)+k*transpose(k)*(1-cos(theta));
end

