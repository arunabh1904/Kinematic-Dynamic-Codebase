%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Function name: quat2Rot
%Returns a rotation matrix R given a 4X1 vector of Quaternions

%[R] = quat2Rot(Q)
%cpMap(): Calls the cross product matrix
%Sq is the real part of the quaternion

%R = the rotation matrix describing rotation for a given roll, pitch, yaw
% Q is the 4X1 quaternion vector

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function R = quat2Rot(Q)
Sq= cpMap(Q(2:4));
R=(Q(1)*Q(1) - dot(Q(2:4),Q(2:4)))*eye(3)+2*Q(1)*Sq+2*Q(2:4)*transpose(Q(2:4));
end
