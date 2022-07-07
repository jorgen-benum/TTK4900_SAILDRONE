function [I_new_pos] = parallelAxisTH(position_vector, mass, I_prev_pos)
% calculate the new inertia matrix at a new position using the parallel axis theorem
% position_vector is the position vector from the position the I_new
% is calulated to the position of the previous inertia

    I_new_pos = I_prev_pos + mass*(transpose(position_vector)*position_vector*eye(3) - position_vector*transpose(position_vector));

end

