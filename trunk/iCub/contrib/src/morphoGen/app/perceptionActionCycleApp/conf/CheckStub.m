clear
clc
close all
load ThePresent.txt
load WeightNOPres.txt
Weight=WeightNOPres;

epRpc=zeros(20,50)
%        epRpc(1,:)=ThePresent(1,:);
       epRpc(2,:)=ThePresent(2,:);
%        epRpc(3,:)=ThePresent(3,:);
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