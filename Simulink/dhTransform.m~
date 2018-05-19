%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Function name: dhTransform
%Returns transformation matrix for a given set of DH parameters
%
%[ H ] = dhTransform( a,d,alpha,theta )
%
%[H] = the homogenous transformation matrix describing transformation 
%
%[a] = fixed link length/the perpendicular distance between two consecutive 
%Z axis along X axis in meters
%
%[alpha] = fixed angle between links/the angle between two consecutive Z axis
%about X axis (in radians)
%
%[d] = the perpendicular distance between two consecutive X axis along Z axis
%or the variable length if link in case of a prismatic joint in meters
%
%[theta] = the angle between two consecutive X axis about Z axis or the
%variable angle between links in case of a revolute joint (in radians)
%
%Name: Vineet Pandey
%CWID: 10826588 
%Course Number: MEGN544
%Date: 09/29/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ H ] = dhTransform( a,d,alpha,theta )
Rx = rotX(alpha);
Rz = rotZ(theta);
T1 = [Rz [0 0 d]';0 0 0 1];
T2 = [Rx [a 0 0]';0 0 0 1];


H= T1*T2;

end