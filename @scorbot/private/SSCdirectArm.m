function [uTs,Q]=SSCdirectArm(scbconsts,thetas)
% Return into uTs a cell with 4 elements, each one the transformation 
% matrix for going from a link to the universal frame:
%   1-> univT0, 2->univT1, 3->univT2, 4->univT3
% SCBCONSTS and THETAS are as in "SSClinkT()"
% Return into Q the position x,y,z of the end of segment 3 in the universal
% system in m. 

    Ts=cell(1,4); % 1-> univT0, 2->0T1, 3->1T2, 4->2T3
    for (f=0:3)
        Ts{f+1}=SSClinkT(f,scbconsts,thetas);
    end
    
    uTs=cell(1,4); 
    for (f=1:4)
        uTs{f}=Ts{1};
        for (g=2:f)
            uTs{f}=uTs{f}*Ts{g};
        end
    end
    
    utip=uTs{4}*[scbconsts.l3;0;0;1];
    Q=utip(1:3);
        
end