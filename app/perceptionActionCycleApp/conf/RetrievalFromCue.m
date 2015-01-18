function[mRet]=RetrievalFromCue(fign,T,u)

% cnt=1
% 
% for i=1:20
%     for j=1:50
%     pic(i,j)=u(1,cnt);
%     cnt=cnt+1;
%     end
% end
% imshow(pic,'InitialMagnification', 1000)
% 
% figure(fign+20);imagesc(T);colormap gray;grid on  % displays T
% xlabel('Tij  matrix')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
kkk=0;
V=u;
delt=0.0015;        

    for j=2:5000 %Euler integration loop
        inhibition =    -30 + 3.5*sum(V);% was 3.5
        if(inhibition<0);inhibition=0;end
        u = u +delt*(-u/100 + V*T - inhibition);
        % -3.5 cut out
        V=u;
        V(find(V<0))=0;
    end

mRet=V;
