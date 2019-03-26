function pos=searchStr(str1,str2)
% find STR1 into STR2

    pos=strfind(str2,str1);
    if (isempty(pos)) 
        pos=0;
    else
        pos=pos(1);
    end

end
