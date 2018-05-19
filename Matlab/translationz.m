function [ T ] = translationz( z )
%TRANSZ Summary of this function goes here
%
% [output1, output2] = myFunctionName(input1, input2) Now a more description 
%  multiline description of the function would be appropriate.
% 
% output1 = description of what the first output is/means include units if appropriate
% output2 = description of what the second output is/means include units if appropriate
% 
% input1 = description of what the first input is/means include units if appropriate
% input2 = description of what the second input is/means include units if appropriate


T = [1 0 0 0; 0 1 0 0; 0 0 1 z; 0 0 0 1];

end

