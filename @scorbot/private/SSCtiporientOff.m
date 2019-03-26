function newdisplay=SSCtiporientOff(display)
% From this moment until the tip orientation is set to on, the display 
% does not show the tip orientation

    newdisplay=display;
    newdisplay.tiporient=0; 

end