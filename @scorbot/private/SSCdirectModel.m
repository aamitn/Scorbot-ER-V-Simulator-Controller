function [uTs,Q,uTg,posetip]=SSCdirectModel(scbconsts,thetas,pitch,roll)
% Calculate the direct kinematic model of the entire robot, given the three
% theta angles in THETAS (rads, as in Denavit-Hatenberg), the PITCH and the
% ROLL (rads, in the universal frame). 
%
% UTS: a cell with 4 homogeneous transforms: 1-> from 0 to univ, 2-> from
% 1 to univ, 3-> from 2 to univ, 4-> from 3 to univ
% Q: coordinates x,y,z in the univ. frame of the end point of forearm
% UTG: a homogeneous transform from the gripper reference frame to univ.
% POSETIP: elements 1-3-> x,y,z in the univ. frame of the tip of the
% gripper; elements 4-6-> yaw, pitch and roll of the gripper w.r.t. the
% univ. frame

     [uTs,Q]=SSCdirectArm(scbconsts,thetas);
     [uTg,posetip]=SSCdirectGripper(scbconsts,Q,thetas(1),pitch,roll);
     
end