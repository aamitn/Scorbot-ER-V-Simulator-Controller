function T=SSClinkT(i,scbconsts,thetas)
% Return the homogeneous transform (i-1)T_i for the scorbot, being i in
% [0,3]. If i==0, return the transform for the universal frame
% SCBCONSTS is a struct with constants that define the size of the robot:
% l0, l1, l2, l3 (all in meters)
% THETAS is a vector with the values, in radians, of the joints (1-3)
% according to the D-H model (see angulosDH1/2.jpg).

    if (~isscalar(i))||(~isnumeric(i))
        error('I must be a scalar');
    end
    if (i~=floor(i))
        error('I must be an integer');
    end
    if (i<0)||(i>3)
        error('I out of range');
    end
    if (~isvector(thetas))||(length(thetas)~=3)||(~isreal(thetas))
        error('Invalid thetas vector');
    end
    
    switch (i)
        case 0
            T=[1 0 0 0 ; 0 1 0 0 ; 0 0 1 scbconsts.l0; 0 0 0 1];
        case 1
            c1=cos(thetas(1));
            s1=sin(thetas(1));
            T=[c1 -s1 0 0; s1 c1 0 0; 0 0 1 0; 0 0 0 1];
        case 2
            c2=cos(thetas(2));
            s2=sin(thetas(2));
            T=[c2 -s2 0 scbconsts.l1; 0 0 -1 0; s2 c2 0 0 ; 0 0 0 1];
        case 3
            c3=cos(thetas(3));
            s3=sin(thetas(3));
            T=[c3 -s3 0 scbconsts.l2; s3 c3 0 0; 0 0 1 0 ; 0 0 0 1];
    end

end