function value = bezierd( afra, s )
[n, m] = size(afra);
value=zeros(n,1);
M = m-1;
if M==3
    k=[3 6 3];
elseif M==4
    k=[4 12 12 4];
elseif M==5
    k=[5 20 30 20 5];
elseif M==6
    k=[6 30 60 60 30 6];
else
    return;
end
%%
x = ones(1, M);
y = ones(1, M);
for i=1:M-1
    x(i+1)=s*x(i);
    y(i+1)=(1-s)*y(i);
end
for i=1:n
   value(i) = 0;
   for j=1:M
      value(i) = value(i) + (afra(i,j+1)-afra(i,j))*k(j)*x(j)*y(M+1-j);
   end
end


  