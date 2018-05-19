function [Jv, JvDot] = velocityJacobian(paramList, paramRateList)
%VELOCITYJACOBIAN Summary of this function goes here
%
% [Jv, JvDot] = velocityJacobian(linkList, paramList,paramRateList) Now a more description
%  multiline description of the function would be appropriate.
%
% Jv = velcoity jacobian
% Jvdot = velocity hessian
%
% linkList = the current joint angles/distances. (an Nx1 array)
% paramList= the current joint angle/distance speeds. (an Nx1 array)
%

%% Initialization of variables

N=3;
a = zeros(N,1);
d = zeros(N,1);
alpha = zeros(N,1);
theta = zeros(N,1);
isRotary = zeros(N,1);
N=3;
d_dot = zeros(N,1);
theta_dot = zeros(N,1);
T=zeros(4,4,N);
JvDot = zeros(6,3);
Jv=zeros(6,3);
li1=[0.03091,0.03091,6.625e-3];
li2=[0.0265,1.7799,1.7799];
li3=[0.01325,0.2275,0.2275];
linkList=repmat(createLink(0,0.25,pi/2,[],[0;0.125;0],5.30,diag(li1)),[N,1]);
linkList(1)=createLink(0,0.25,pi/2,[],[0;0.125;0],5.30,diag(li1));
linkList(2)=createLink(1,0,0,[],[-0.5;0;0],21.20,diag(li2));
linkList(3)=createLink(0.5,0,0,[],[-0.5;0;0],10.60,diag(li3));
dd = zeros(3,N+1);
Z = zeros(3,N+1);
w = zeros(3,N+1);
dd_dot = zeros(3,4);
H = eye(4);

for i = 1:N
    a(i) = linkList(i).a;
    alpha(i) = linkList(i).alpha;
    isRotary(i) = linkList(i).isRotary;
    %     if isRotary(i) == 0                     % For Prismatic
    %         theta(i) = linkList(i).theta;
    %         d(i) = paramList(i);
    %         if exist('paramRateList','var')
    %             d_dot(i) = paramRateList(i);
    %         end
    %     else   % Rotary
    d(i) = linkList(i).d;
    theta(i) = paramList(i);
%     if exist('paramRateList','var')
        theta_dot(i) = paramRateList(i);
        %         end
%     end
end

for i = 1:N
    T(:,:,i) = zeros(4,4);
end

for i = 1:N
    T(:,:,i)=H*dhTransform(a(i),d(i),alpha(i),theta(i));
    H = T(:,:,i);
end



dd(:,1:1) = [0; 0; 0];
Z(:,1) = [0 0 1];
w(:,1:1) = [0; 0; 0];
dd_dot(:,1:1) = [0; 0; 0];
for i = 2:N+1
    dd(:,i:i) =  T(1:3,4:4,i-1);
    Z(:,i) = T(1:3,3:3,i-1);
    if isRotary(i-1) == 1
        w(:,i) = theta_dot(i-1)*Z(:,i-1)+w(:,i-1);
    else
        w(:,i) = w(:,i-1);
    end
    if isRotary(i-1) == 1
        dd_dot(:,i) = dd_dot(:,i-1) + cross(w(:,i),(dd(:,i)-dd(:,i-1)));
    else
        dd_dot(:,i) = dd_dot(:,i-1) + cross(w(:,i),(dd(:,i)-dd(:,i-1))) + d_dot(i-1)*Z(:,i-1);
    end
    
end

%% Velocity Jacobain
for i = 1:N
    if isRotary(i) == true            % Rotary
        Jv(1:3,i:i) = cpMap(Z(:,i))*(T(1:3,4,N)-dd(:,i));
        Jv(4:6,i:i) = Z(:,i);
%     else       % Prismatic
%         Jv(1:3,i:i) = Z(:,i);
%         Jv(4:6,i:i) = [0; 0; 0];
    end
end

%% Velcoity hessian (Jv_dot)



% if exist('paramRateList','var')
for i = 1:N
    if isRotary(i) == 1             % Rotary
        JvDot(:,i) = [cross(cross(w(:,i),Z(:,i)),(dd(:,N+1)-dd(:,i)))+cross(Z(:,i),dd_dot(:,4)-dd_dot(:,i)); cross(w(:,i),Z(:,i))];
        %         else       % Prismatic
        %             JvDot(:,i) = [cross(w(:,i),Z(:,i)); 0; 0; 0];
        %         end
        
    else
        JvDot = [];
    end
end
