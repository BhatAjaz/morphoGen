wb = formwb(Achilles,Achilles.b,Achilles.IW,Achilles.LW);
[b,iw,lw] = separatewb(Achilles,wb);
b1 = cell2mat(b(1,:));
b2 = cell2mat(b(2,:));
b3 = cell2mat(b(3,:));
w1 = cell2mat(iw(1,:));
w2 = cell2mat(lw(2,1));
w3 = cell2mat(lw(3,2));
size(b1)
size(b2)
size(b3)
size(w1)
size(w2)
size(w3)
save w1.txt w1 -ascii
save w2.txt w2 -ascii
save w3.txt w3 -ascii
save b1.txt b1 -ascii
save b2.txt b2 -ascii
save b3.txt b3 -ascii
