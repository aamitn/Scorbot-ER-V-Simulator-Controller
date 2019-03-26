function res=ACLsetauxpos(s,pos,mode)

    res=0;
    if (mode=='j')
        for (f=1:5)
            [rescode,restxt]=ACLcommand(s,sprintf('setpv pmcl0 %d %d',f,pos.joints(f)),0);
            if (rescode~=1)
                fprintf('Error setting auxiliary position [%s].\n',restxt);
                return;
            end
        end
    elseif (mode=='x')
        names='xyz';
        for (f=1:3)
            [rescode,restxt]=ACLcommand(s,sprintf('setpvc pmcl0 %s %d',names(f),pos.xyz(f)),0);
            if (rescode~=1)
                fprintf('Error setting auxiliary %s position [%s].\n',names(f),restxt);
                return;
            end
        end
		% the following restore pitch and roll after changing cartesian coords, since the latter changes those angles
        [rescode,restxt]=ACLcommand(s,sprintf('setpvc pmcl0 P %d',pos.pitch),0);
        if (rescode~=1)
            fprintf('Error setting auxiliary pitch position [%s].\n',restxt);
            return;
        end
        [rescode,restxt]=ACLcommand(s,sprintf('setpvc pmcl0 R %d',pos.roll),0);
        if (rescode~=1)
            fprintf('Error setting auxiliary roll position [%s].\n',restxt);
            return;
        end
    elseif (mode=='p')
		fprintf('Changing only pitch is still under construction!\n');
		return; % because I've detected that changing the pitch also changes other axes

        [rescode,restxt]=ACLcommand(s,sprintf('setpvc pmcl0 P %d',pos.pitch),0);
        if (rescode~=1)
            fprintf('Error setting pitch position [%s].\n',restxt);
            return;
        end        
    elseif (mode=='r')
        [rescode,restxt]=ACLcommand(s,sprintf('setpvc pmcl0 R %d',pos.roll),0);
        if (rescode~=1)
            fprintf('Error setting roll position [%s].\n',restxt);
            return;
        end        
    end
    res=1;

end
