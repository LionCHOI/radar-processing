function quat = singleAxisRotation(quat_in, x_deg, y_deg, z_deg)
    % perform rotation operation along a single axis and return the updated quaternion
    % input angle should be in [deg]
    
    rot_angles = [x_deg, y_deg, z_deg];
    if sum(rot_angles ~= 0) > 1
        error('rotation operation can be performed for a single axis');
    end

    rot_axis = find(rot_angles ~= 0);
    if isempty(rot_axis)
        return;
    end
    rot_angle = rot_angles(rot_axis) * pi/180;
    new_quat = [cos(rot_angle/2), 0, 0, 0];
    new_quat(rot_axis + 1) = sin(rot_angle/2);
    
    quat = quatmultiply(quat_in, new_quat);
end