
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Function name: rpy2Rot
%Returns rotation matrix describing rotation for a given roll, pitch yaw
%[R] = rpy2ROT(roll,pitch,yaw)
%R = the rotation matrix describing rotation for a given roll, pitch, yaw
%roll = angle at which rotation is made about X axis in radians
%pitch = angle at which rotation is made about Y axis in radians
%yaw = angle at which rotation is made about Z axis in radians
% rz= rotation matrix for rotation about z axis
% ry= rotation matrix for rotation about y axis
% rx= rotation matrix for rotation about x axis

%Name: Vineet Pandey
%CWID: 10826588
%Course Number: MEGN544
%Date: 09/29/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function R = rpy2Rot(roll,pitch,yaw)
rz= rotZ(roll);
ry= rotY(pitch);
rx= rotX(yaw);
R=rz*ry*rx;
end