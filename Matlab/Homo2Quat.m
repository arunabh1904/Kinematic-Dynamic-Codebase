function [  Quat_Transform ] = Homo2Quat( H )
%HOMO2QUAT returns Quaternion in [P,Q]' format


    K_temp = rot2Quat(H(1:3,1:3));
    d_des = H(1:3,4); % position_desired_ distance part 
    d=d_des;
    K_temp=reshape(K_temp,[4,1]);
    K_temp=K_temp/norm(K_temp);
    Quat_Transform=[d;K_temp];
end

