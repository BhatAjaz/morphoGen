clear
clc
close all
load WeightNOPres.txt
load E770.txt
Weight=WeightNOPres;

i=1;
NewEM=Unfold2by2(E770(10,:));
figure(i)
i=i+1;
imshow(NewEM,'InitialMagnification', 1000);

epRpc=zeros(20,50)

      epRpc(1,:)=NewEM(1,:);
       epRpc(2,:)=NewEM(2,:);
figure(i)
subplot(2,1,1)
imshow(epRpc,'InitialMagnification', 1000)
epp=Unfold2to1(epRpc);
xxx=RetrievalFromCue(1,Weight,epp);
subplot(2,1,2)
imshow(Unfold2by2(xxx),'InitialMagnification', 1000)
i=i+1;


% Testex2
NewEM=Unfold2by2(E770(8,:));
figure(i)
i=i+1;
imshow(NewEM,'InitialMagnification', 1000);

epRpc=zeros(20,50)

%       epRpc(1,:)=NewEM(1,:);
       epRpc(2,:)=NewEM(2,:);
figure(i)
subplot(2,1,1)
imshow(epRpc,'InitialMagnification', 1000)
epp=Unfold2to1(epRpc);
xxx=RetrievalFromCue(1,Weight,epp);
subplot(2,1,2)
imshow(Unfold2by2(xxx),'InitialMagnification', 1000)
i=i+1;


% TestEx3
NewEM=Unfold2by2(E770(1,:));
figure(i)
i=i+1;
imshow(NewEM,'InitialMagnification', 1000);

epRpc=zeros(20,50)

      epRpc(7,:)=NewEM(7,:);
       epRpc(8,:)=NewEM(8,:);
figure(i)
subplot(2,1,1)
imshow(epRpc,'InitialMagnification', 1000)
epp=Unfold2to1(epRpc);
xxx=RetrievalFromCue(1,Weight,epp);
subplot(2,1,2)
imshow(Unfold2by2(xxx),'InitialMagnification', 1000)
i=i+1;

