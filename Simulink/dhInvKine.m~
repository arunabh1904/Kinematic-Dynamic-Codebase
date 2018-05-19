% dhInvKine: Returns the parameter list necessary to achieve a desired homogenous transform 
% and the residual error in that transform.
% 
% [paramList, error] = dhInvKine (linkList, desTransform, paramListGuess)
% 
% output1 = residual error in the transform
% output2 = parameter list necessary to achieve desired homogenous transform
%
% input1 = linkList(the list of joint parameters created with createLink)
% input2 = desired homogenous transform(4-by-4)
% input3 = paramListGuess(initial guess at the parameters)

% Vineet Pandey
% 10826588

function [paramList, error] = dhInvKine (linkList, desTransform, paramListGuess)
% Argument management:
if nargin < 4
    error_des=0.001*ones(7,1);
else
    error_des=error_des*ones(7,1);
end

    paramListGuess=reshape(paramListGuess,[length(paramListGuess),1]);
    
    Tqdes=Homo2Quat(desTransform);
    Tqur=Homo2Quat(dhFwdKine(linkList,paramListGuess));
    
    error=Tqdes-Tqur;
    i=0;
    iter_num=1000;
    while (norm(error)>norm(error_des)) && (i<iter_num) ,
q=Tqur(5:end); % q_vectors
        q_o=Tqur(4);   % q_knot
        c=(1/2)*[-q';...
            q_o*eye(3)-cpMap(q)];
        Jv_c =velocityJacobian(linkList,paramListGuess);
        J_g=[eye(3),zeros(3);...
            zeros(4,3),c]*Jv_c;
     
        d_q=pinv(J_g)*error;  % make it positive and without gian now working in 4 itr
       
        paramListGuess=paramListGuess+[d_q];
        Tqur=Homo2Quat(dhFwdKine(linkList,paramListGuess));
        error = (Tqdes-Tqur);
        i=i+1;
    end

    if i >= iter_num,paramList=NaN;
        error=inf;
        error('Solution is not aviable with this method'); 
    else
        paramList=paramListGuess;
        error=norm(error);
    end


end

