%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Function name: rot2RPY
%Returns roll, pitch, yaw for a given rotation matrix
%[roll, pitch, yaw] = rot2RPY(R)
%input 3x3 matrix is taken and checked for singularity. Singularity exists
%if pitch = 0. An if else clause is applied and if singularity exists, it is
%mentioned out. Else, RPY angles are evaluated and printed
%costheta1 is the square of cos(theta) obtained
%costheta2 is a 2X1 vector giving the possible values od cos(theta) 
%R = the rotation matrix describing rotation about X axis for a given theta
%Name: Vineet Pandey
%CWID: 10826588
%Course Number: MEGN544
%Date: 09/29/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [roll, pitch, yaw] = rot2RPY(R)
costheta1= R(1,1)*R(1,1)+R(2,1)*R(2,1);
costheta2=[abs(sqrt(costheta1)), -1*abs(sqrt(costheta1))];



if abs(sqrt(costheta1))==0
    
        for i = 1:2
    
        pitch(i,1) = pi/2;
        yaw(i,1)  = 0;
        roll(i,1)   = -atan2(R(1,2), R(2,2));
        
        end
        
        disp(roll);
        disp(pitch);
        disp(yaw);
       
   
else    
    for i = 1:2
    
        pitch(i,1) = atan2(-R(3,1), costheta2(i));
        yaw(i,1)  = atan2(R(3,2)/costheta2(i), R(3,3)/ costheta2(i));
        roll(i,1)   = atan2(R(2,1)/costheta2(i), R(1,1)/costheta2(i));
        
        
    end 
        disp(roll);
        disp(pitch);
        disp(yaw);
end    
end 

