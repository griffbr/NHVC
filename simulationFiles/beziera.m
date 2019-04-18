function value = beziera( afra, s )
[n, m] = size(afra);
value=zeros(n,1);
M = m-1;
if M==3
    k=[6 6];
elseif M==4
    k=[12 24 12];
elseif M==5
    k = [20 60 60 20];
elseif M==6
    k=[30 120 180 120 30];
else
    return;
end
%%
x = ones(1, M-1);
y = ones(1, M-1);
for i=1:M-2
    x(i+1)=s*x(i);
    y(i+1)=(1-s)*y(i);
end
for i=1:n
   value(i) = 0;
   for j=1:M-1
      value(i) = value(i) + (afra(i,j+2)-2*afra(i,j+1)+afra(i,j)) *k(j)*x(j)*y(M-j);
   end
end


  