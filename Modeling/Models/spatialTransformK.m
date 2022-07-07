function [K__a_b] = spatialTransformK(R__a_b, r__b_ba, type)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
% type = 'force'
O3x3 = zeros(3,3);

if strcmp(type, 'motion')
K__a_b = [R__a_b,       -R__a_b*Smtrx(r__b_ba);
          O3x3,         R__a_b];
elseif strcmp(type, 'force')
K__a_b = [R__a_b,                   O3x3;
          -R__a_b*Smtrx(r__b_ba),    R__a_b];

% if strcmp(type, 'motion')
% K__a_b = [R__a_b,       Smtrx(r__a_ab)*R__a_b;
%           O3x3,         R__a_b];
elseif strcmp(type, 'force-r__a_ab') 
% use vector from next frame instead of vector instead of vector in the frame we transform from
r__a_ab = r__b_ba; % input is r__a_ab vector not r__b_ba

K__a_b = [R__a_b,                   O3x3;
          Smtrx(r__a_ab)*R__a_b,    R__a_b];
else
    disp('type is not a valid type, must be "force", force-r__a_ab or "motion"');
    k % set variable to crash
end

end

