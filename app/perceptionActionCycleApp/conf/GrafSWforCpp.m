clear all
clc
close all
load HubA.txt
load MapA.txt
load AcInitActiv.txt
load PXper.txt
load ThePresent.txt
load PCue.txt

LocalNeu=MapA;
u=HubA;
figC=1;
%  Plot Color ========================
cnt=1
for i=1:3
    for j=1:3
        colNurAct(i,j)=LocalNeu(cnt);
        cnt=cnt+1;
    end
end

subplot(2,2,figC)
figC=figC+1;
imshow(colNurAct,'InitialMagnification', 100000)
colormap(Summer)
xlabel('Color SOM activity','FontSize',12)

%  Plot Shape ========================
cnt=1
for i=1:5
    for j=1:3
        colNurAct(i,j)=LocalNeu(cnt+60);
        cnt=cnt+1;
    end
end
subplot(2,2,figC)
figC=figC+1;
imshow(colNurAct,'InitialMagnification', 100000)
colormap(Summer)
xlabel('Shape SOM activity','FontSize',12)
  
%  Plot Word ========================  
  cnt=1
for i=1:5
    for j=1:5
        WorNurAct(i,j)=LocalNeu(cnt+30);
        cnt=cnt+1;
    end
end

subplot(2,2,figC)
figC=figC+1;
imshow(WorNurAct,'InitialMagnification', 10000)
colormap(Summer)
xlabel('word SOM activity','FontSize',12)

%  Plot Provincial Hub ========================
cnt=1
for i=1:6
    for j=1:6
        Prov(i,j)=u(cnt);
        cnt=cnt+1;
    end
end

subplot(2,2,figC)
 figC=figC+1;
imshow(Prov,'InitialMagnification', 10000)
colormap(Summer)
xlabel('Hub SOM activity','FontSize',12)


%  Plot Action ========================
cnt=1
for i=1:3
    for j=1:4
        ActiN(i,j)=AcInitActiv(cnt);
        cnt=cnt+1;
    end
end

figure(14)
imshow(ActiN,'InitialMagnification', 100000)
colormap(Summer)
xlabel('Color SOM activity','FontSize',12)

figure(140)
imshow(Unfold2by2(PXper'),'InitialMagnification', 1000)
figure(141)
imshow(Unfold2by2(PCue'),'InitialMagnification', 1000)

load ThePresent.txt
figure(140)
imshow(ThePresent,'InitialMagnification', 1000)


