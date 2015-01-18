clear
clc
close all
load ThePresent.txt
load Episodes77N.txt
load WMems77N.txt
Epis=9;
Episodes77O=zeros(25,1000);
for i=1:Epis
Episodes77O(i,:)=Episodes77N(i,:);
end
Episodes77O(7,1000)=1;
figure(1)
imshow(ThePresent,'InitialMagnification', 1000)
[m n]=size(ThePresent)
NewEpisode=zeros(20,50);
ctrr=1;
for i=1:m
NewEpisode(i,:)=ThePresent(ctrr,:);
ctrr=ctrr+1;
end
figure(2)
imshow(NewEpisode,'InitialMagnification', 1000);
NewEunfolded=Unfold2to1(NewEpisode);
Episodes77O(Epis+1,:)=NewEunfolded;
NewEM=Unfold2by2(Episodes77O(Epis+1,:));
figure(3)
imshow(NewEM,'InitialMagnification', 1000);
TNew=MemMaint(Episodes77O);
fid = fopen('WeightNOPres.txt','wt');

        for ii = 1:size(TNew,1)
            fprintf(fid,'%g\t',TNew(ii,:));
            fprintf(fid,'\n');
        end
        fclose(fid)
        
        
        fid = fopen('E770.txt','wt');

        for i2 = 1:size(Episodes77O,1)
            fprintf(fid,'%g\t',Episodes77O(i2,:));
            fprintf(fid,'\n');
        end
        fclose(fid)

figure(32)
imshow(WMems77N,'InitialMagnification', 1000)
figure(45)
imshow(TNew,'InitialMagnification', 1000)


load WhepisN.txt
Whubu=WhepisN';
for ik=1:20
pk=NewEpisode(ik,:)
if(pk(48)==1)
for jk=1:42
if(pk(jk)==1)
Whubu(jk,((ik-1)*50)+jk)=1;
end
end
end
end
WhepNtr=Whubu';
WhepNtr(74,24)

 fid = fopen('WHubEpNew.txt','wt');

        for i2 = 1:size(WhepNtr,1)
            fprintf(fid,'%g\t',WhepNtr(i2,:));
            fprintf(fid,'\n');
        end
        fclose(fid)

