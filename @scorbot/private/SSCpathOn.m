function newdisplay=SSCpathOn(display)
% From this moment until the path is set to off, the display stores all the
% cartesian positions of the robot and displays them

    newdisplay=display;
    newdisplay.pathon=1; % if previous path, it is kept

end