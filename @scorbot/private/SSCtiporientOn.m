function newdisplay=SSCtiporientOn(display)
% From this moment until the tip orientation is set to off, the display 
% shows the tip orientation

    newdisplay=display;
    newdisplay.tiporient=1; 

end