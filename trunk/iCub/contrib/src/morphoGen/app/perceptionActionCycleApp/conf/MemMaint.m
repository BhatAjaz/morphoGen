function [WMaint]=MemMaint(Epies)

N=1000;
T=zeros(N,N);

for k=1:N
   for kk=1:N
        for j=1:findNumActive(Epies,25)
            T(k,kk)=T(k,kk)+Epies(j,k)*Epies(j,kk);
        end
        if(k==kk);T(k,kk)=0;end
    end
end

T=sign(T);
WMaint=T;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure(100);imagesc(T);colormap gray;grid on  % displays T
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%