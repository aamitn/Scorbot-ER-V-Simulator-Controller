function [res,pos]=ACLgetauxpos(scb)

    res=0;
    pos=struct();
    [rescode,restxt]=ACLcommand(scb,'listpv pmcl0',0);
    if (rescode~=1)
        fprintf('Error: %s\n',restxt);
        return;
    else
        poscolons=strfind(restxt,':');
        if (length(poscolons)~=10)
            fprintf('Error interpreting position (1)\n');
            return;
        end
        [joints,valid]=ACLextractNvalsfrompos(restxt,poscolons(1:5));
        if (~valid)
            return;
        end
        [xyz,valid]=ACLextractNvalsfrompos(restxt,poscolons(6:8));
        if (~valid)
            return;
        end
        [pr,valid]=ACLextractNvalsfrompos(restxt,poscolons(9:10));
        if (~valid)
            return;
        end
        pos=struct('joints',joints,...
                   'xyz',xyz,...
                   'pitch',pr(1),...
                   'roll',pr(2));
        res=1;
    end
 
end