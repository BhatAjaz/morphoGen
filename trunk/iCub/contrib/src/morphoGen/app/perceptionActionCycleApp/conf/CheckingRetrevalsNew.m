load W4Mems.txt
Weight=W4Mems;

epRpc=zeros(20,50)
for i=1:2
       epRpc(i,:)=RakeAssemblySchema(i,:);
end
% epRpc(4,:)=ReachFailConsolidateRev(4,:);
subplot(2,1,1)
imshow(epRpc,'InitialMagnification', 1000)
epp=Unfold2to1(epRpc);
xxx=RetrievalFromCue(1,Weight,epp);
subplot(2,1,2)
imshow(Unfold2by2(xxx),'InitialMagnification', 1000)