function newdisplay=SSCpathOff(display)
% From this moment until the path is set to on, the display does not store
% the cartesian positions of the robot 

    newdisplay=display;
    newdisplay.pathon=0;
    newdisplay.path=[];

end