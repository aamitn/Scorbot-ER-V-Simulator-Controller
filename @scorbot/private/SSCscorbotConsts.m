function scbconsts=SSCscorbotConsts()
% Return the constant values that define the scorbot robot
% see acl ref guide - parameter section and er V manual for some of these values

    scbconsts=struct('l0',0.19+0.14,...
                     'l1',0.015,...
                     'l2',0.2211,... 
                     'l3',0.2211,...
                     'minjoints',[-155 -35 -60 -130 -180],... % minimum values in degrees of each joint of the real robot
                     'maxjoints',[155 130 100 130 180],... % max values
                     'axis1resol',90/3831,... % parameter 33 of ACL, scorbot er-v (er-v plus are different)
                     'axis2resol',90/3065,... % parameter 34
                     'axis3resol',90/3065,... % parameter 35
                     'hgripper',0.075/2,... % space between a tip of the gripper and its center when completely opened
                     'lgripper',0.1431,... % from rotation axis to tip of the gripper
                     'l2gripper',0.07,... % from tip of the gripper to rigid part of the gripper
                     'wgripper',0.015,... % width one of the solid gripper tips
                     'ang0shoulder',pi/4+pi/2,...
                     'baseradius',0.075, ...
                     'baseheight',0.21,...
                     'bodywidth',0.20,... % or diameter around axis 1
                     'bodydepth',0.20,... % or diameter around axis 1
                     'bodyheight',0.14,... % from bottom to axis 2
                     'armwidth',0.06 ...
                     );
        
end
