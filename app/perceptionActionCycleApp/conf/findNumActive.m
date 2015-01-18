function [m]=findNumActive(VecF,sizz)
m=0;
for i=1:sizz
    if(sum(VecF(i,:))==0)
        m=i-1;
       break
    end
end