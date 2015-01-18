clear
clc
close all
load WMems7.txt
load PCue.txt
load HeEpMult.txt
load MinEnergyPlan.txt
load UsefulAcSeqs.txt
load HubTDCom.txt

[m,n] = size(PCue)  
for i=1:m
figure(72+i)    
imshow(Unfold2by2(PCue(i,:)),'InitialMagnification', 1000)
end

figure(32)
T=WMems7;
imshow(T,'InitialMagnification', 1000)
    
[m,n] = size(HeEpMult)  
for i=1:m
figure(22+i)    
imshow(Unfold2by2(HeEpMult(i,:)),'InitialMagnification', 1000)
end

figure(55)
imshow(Unfold2by2(MinEnergyPlan),'InitialMagnification', 1000)


figure(22)
imshow(Unfold2by2(PXper),'InitialMagnification', 1000)

for i=1:m
figure(27+i)    
imshow(Unfold2by2(UsefulAcSeqs(i,:)),'InitialMagnification', 1000)
end

for i=1:m
figure(32+i)        
cnt=1;
for i=1:7
    for j=1:6
        ProvJJ(i,j)=HubTDCom(i,cnt);
        cnt=cnt+1;
    end
end

imshow(ProvJJ,'InitialMagnification', 5000)
colormap(Summer)

end


for i=1:7
figure(27+i)    
imshow(Unfold2by2(Episodes7(i,:)),'InitialMagnification', 1000)
end

