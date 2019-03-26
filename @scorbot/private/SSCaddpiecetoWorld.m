function newworld=SSCaddpiecetoWorld(world,piece)
% Add a new piece to the existing world. PIECE is a struct as follows:
%   length,width,depth -> dimensions of the piece on the univ. axes x, y
%   and z respectively.
%   x,y-> in universal coordinates: where the center of
%   the face that touches the ground is placed.
%   c-> color (just one character)

    newp=struct('xsize',piece.length,...
                'ysize',piece.width,...
                'zsize',piece.depth,...
                'T',[1 0 0 piece.x; 0 1 0 piece.y ; 0 0 1 piece.depth/2],...
                'c',piece.c);
    newworld=world;
    newworld.pieces{length(world.pieces)+1}=newp;

end