%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Function name: rot2Quat
%Returns Quaternions in the form of a 4X1 vector

%[Q] = rot2Quat(R)

%R = the rotation matrix describing rotation for a given roll, pitch, yaw
% q1,q2,q3 are the real vector components
% q0 is the imaginary component

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Q = rot2Quat(R)
q0=0.5*abs(sqrt(1+trace(R)));
if q0~=0
    q=(1/(4*q0))*[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];
    modq=norm(q);
    theta=2*atan2(modq,q0);
    Q=[q0;q(1);q(2);q(3)];
else
     theta= pi;
     q1=abs(sqrt(0.25*(1+R(1,1)-R(2,2)-R(3,3))));
     q2=abs(sqrt(0.25*(1-R(1,1)+R(2,2)-R(3,3))));
     q3=abs(sqrt(0.25*(1-R(1,1)-R(2,2)+R(3,3))));
     Q=[0;q1;q2;q3];
end    
end

