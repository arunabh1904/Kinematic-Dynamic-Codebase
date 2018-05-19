%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Function name: cpMap
%Returns cross product matrix for a vector

%[X] = cpMap(w)

%X = the cross product matrix

%w = three dimensional input vector. Input is given as a column matrix.



function [X] = cpMap(w)

X = [0 -w(3,1) w(2,1); w(3,1) 0 -w(1,1); -w(2,1) w(1,1) 0];
end
