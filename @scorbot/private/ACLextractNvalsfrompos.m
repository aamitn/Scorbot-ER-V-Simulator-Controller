function [vals,valid]=ACLextractNvalsfrompos(txt,poscolons)

    vals=[];
    valid=0;
    n=length(poscolons);
    l=length(txt);
    for (f=1:n)
        pos0=poscolons(f)+1;
        pos1=pos0;
        while (pos1<=l) && (txt(pos1)==32)
            pos1=pos1+1;
        end
        if (pos1>l)
            fprintf('Error interpreting position (2)\n');
            return;
        end
        pos0=pos1;
        pos1=pos0+1;
        while (pos1<=l) && ((txt(pos1)~=32)&&(txt(pos1)~=13)&&(txt(pos1)~=10))
            pos1=pos1+1;
        end
        if (pos1>l)
            fprintf('Error interpreting position (3)\n');
            return;
        end
        vals=[vals str2num(txt(pos0:pos1-1))];
        valid=1;
    end
    
end