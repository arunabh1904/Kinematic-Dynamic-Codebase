%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Function name: rot2AngleAxis
%Returns the rotation axis k, rotation angle theta for a given rotation matrix
%R
%[k,theta] = rot2AngleAxis(R)
%k = the vector axis about which the coordinate frame rotation is performed
%theta = angle with which the rotation is performed in radians
%R = the rotation matrix describing rotation about k axis for a given theta

%Name: Vineet Pandey
%CWID: 10826588
%Course Number: MEGN544
%Date: 09/29/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [k,theta ] = rot2AngleAxis( R )
theta  = acos((trace(R)-1)/2);
if theta == pi
    
    k(1,1)=sqrt((R(1,1)+1)/2);
    k(2,1)=sqrt((R(2,2)+1)/2);
    k(3,1)=sqrt((R(3,3)+1)/2);
else
    k = (1/(2*sin(theta)))*[R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
end
end