
clear
clc
close all
load WeightNOPres.txt
load E770.txt
Weight=WeightNOPres;

NewEM=Unfold2by2(Episodes77N(1,:));
figure(1)
imshow(NewEM,'InitialMagnification', 1000);

epRpc=zeros(20,50)

        epRpc(1,:)=ThePresent(1,:);
        epRpc(2,:)=ThePresent(2,:);
%        epRpc(4,:)=ThePresent(4,:);
%        epRpc(5,:)=ThePresent(5,:);
%        epRpc(4,:)=ThePresent(4,:);
%         epRpc(2,48)=0;
%         epRpc(2,50)=0;

% epRpc(4,:)=ReachFailConsolidateRev(4,:);
subplot(2,1,1)
imshow(epRpc,'InitialMagnification', 1000)
epp=Unfold2to1(epRpc);
xxx=RetrievalFromCue(1,Weight,epp);
subplot(2,1,2)
imshow(Unfold2by2(xxx),'InitialMagnification', 1000)



clear
clc
close all
load WMems77N.txt
load Episodes77N.txt
Weight=WMems77N;

NewEM=Unfold2by2(Episodes77N(8,:));
figure(1)
imshow(NewEM,'InitialMagnification', 1000);

epRpc=zeros(20,50)

        epRpc(1,:)=NewEM(1,:);
        epRpc(2,:)=NewEM(2,:);
%        epRpc(4,:)=ThePresent(4,:);
%        epRpc(5,:)=ThePresent(5,:);
%        epRpc(4,:)=ThePresent(4,:);
%         epRpc(2,48)=0;
%         epRpc(2,50)=0;

% epRpc(4,:)=ReachFailConsolidateRev(4,:);
subplot(2,1,1)
imshow(epRpc,'InitialMagnification', 1000)
epp=Unfold2to1(epRpc);
xxx=RetrievalFromCue(1,Weight,epp);
subplot(2,1,2)
imshow(Unfold2by2(xxx),'InitialMagnification', 1000)

save E770.txt Episodes77O -ascii

clear
clc
close all
load WeightNOPres.txt
load E770.txt
Weight=WeightNOPres;

NewEM=Unfold2by2(E770(10,:));
figure(1)
imshow(NewEM,'InitialMagnification', 1000);


epRpc=zeros(20,50)

      epRpc(1,:)=NewEM(1,:);
       epRpc(2,:)=NewEM(2,:);

subplot(2,1,1)
imshow(epRpc,'InitialMagnification', 1000)
epp=Unfold2to1(epRpc);
xxx=RetrievalFromCue(1,Weight,epp);
subplot(2,1,2)
imshow(Unfold2by2(xxx),'InitialMagnification', 1000)


