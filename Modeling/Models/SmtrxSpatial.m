function [SS] = SmtrxSpatial(spatial_vector)
%   Spatial cross product operator for spatial vectors
%   The operator for motion with spatial vector give: 
%   m_1 x m_2 = SS(m_1)m_2 for spatial motion
%   m x f = -SS(m)^T f for spatial motion and forces

    trans_motion = spatial_vector(1:3);
    rot_motion = spatial_vector(4:6);
    
    O3x3 = zeros(3,3);
    
    SS = [Smtrx(rot_motion),    Smtrx(trans_motion);
          O3x3,                 Smtrx(rot_motion)];
    

end

