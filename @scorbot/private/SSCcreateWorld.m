function world=SSCcreateWorld()
% Create an empty world and return it

    world=struct('piecetaken',0,... % index of piece in the gripper, or 0 if none
                 'pieces',[]); % list of pieces; a cell array actually

end