function display=SSCcreateDisplay(scbconsts,scbinitstate,initworld)
% Create a new graphical context for displaying the simulation of the robot
% If SCBINITSTATE is not empty, draw the robot with that state and uses
% INITWORLD in the drawing; otherwise does no drawing

    h=figure;
    
    clf;
    xlabel('x');
    ylabel('y');
    zlabel('z');
    
    display=struct('handle',h,...
                   'justcreated',1,...
                   'view_az',75,...
                   'view_el',20,...
                   'pathon',0,...
                   'path',[],...
                   'tiporient',0,...
                   'lastscbstate',scbinitstate);

    if (~isempty(scbinitstate))
        display=SSCupdateDisplay(display,scbconsts,scbinitstate,initworld);
    end
               
end