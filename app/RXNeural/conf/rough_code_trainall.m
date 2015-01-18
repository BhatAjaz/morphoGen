
% ++++++++++++++++++++++++++ ROUGH WORK +++++++++++++++++++++++++++++++++++++++++++++++++

clear all
clc
close all
% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
load Proximal.txt
load Distal.txt
% opos=ounusma
innu = Proximal';
ounu=Distal';
save innu.txt innu -ascii
save ounu.txt ounu -ascii
% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

% [pn,meanp,stdp,tn,meant,stdt] = prestd(innu,ounu); %preprocesses the network training set 
[R,Q] = size(innu);
iitst = 2:4:Q;
iival = 2:4:Q;
iitr = [2:4:Q 2:4:Q];
val.P = innu(:,iival); val.T = ounu(:,iival);
test.P = innu(:,iitst); test.T = ounu(:,iitst);
ptr = innu(:,iitr); ttr = ounu(:,iitr);

% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Achilles = newff(minmax(ptr),[32 41 3],{'tansig' 'tansig' 'purelin'},'trainlm')
Achilles.trainParam.show = 25
Achilles.trainParam.mem_reduc =25
Achilles.trainParam.epochs = 100
Achilles.trainParam.goal = 0.0001
Achilles.trainParam.max_fail = 10
[Achilles,tr]=train(Achilles,ptr,ttr,[],[],val,test)

% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

% [R,Q] = size(a);
% iitst = 2:4:Q;
% iival = 2:4:Q;
% iitr = [1:3:Q 1:3:Q];
% val.P = a(:,iival); val.T = b(:,iival);
% test.P = a(:,iitst); test.T = b(:,iitst);
% ptr = a(:,iitr); ttr = b(:,iitr);