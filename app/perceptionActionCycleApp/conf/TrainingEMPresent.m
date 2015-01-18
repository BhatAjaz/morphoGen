clear
clc
load ThePresent.txt
load Episodes77N.txt
Epis=1;
Episodes77O=zeros(25,1000);
Episodes77O(1,:)=Episodes77N(1,:);
figure(1)
imshow(ThePresent,'InitialMagnification', 1000)
[m n]=size(ThePresent)
NewEpisode=zeros(20,50);
for i=1:m
    NewEpisode(i,:)=ThePresent(i,:);
end
figure(2)
imshow(NewEpisode,'InitialMagnification', 1000);
NewEunfolded=Unfold2to1(NewEpisode);
Episodes77O(Epis+1,:)=NewEunfolded;
NewEM=Unfold2by2(Episodes77O(Epis+1,:));
figure(3)
imshow(NewEM,'InitialMagnification', 1000);
% ***************************************************
TNew=MemMaint(Episodes77O); 
save WeightNO.txt TNew -ascii
% ***************************************************

