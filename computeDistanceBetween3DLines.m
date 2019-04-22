%compute minimum distance between two 3D lines
%A and B form a line
%C and D form a line
function minimumDistance = computeDistanceBetween3DLines(A, B, C, D)
    
    %get vector betwen A & B and C & D
    vectorAB = [B(1) - A(1), B(2) - A(2), B(3) - A(3)]
    vectorCD = [D(1) - C(1), D(2) - C(2), D(3) - C(3)]
    
    L1=rand(2,3);
    L2=rand(2,3);
    
    L1 = [A;B];
    L2 = [C;D];
    
    [minimumDistance Pc Qc] = distBW2lines(L1,L2)

    