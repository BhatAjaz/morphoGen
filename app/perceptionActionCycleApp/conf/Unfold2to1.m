function [V]=Unfold2to1(MemV)
cv=1;
for i=1:20
    for j=1:50
        V(1,cv)=MemV(i,j);
        cv=cv+1;
    end
end