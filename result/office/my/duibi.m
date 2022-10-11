a=my(1239:2305,1);
b=my(1239:2305,2);
for i=1:(2305-1239)
    a(i)=a(i)-4.0363;
end   
plot(a,b,'-k');
% plot([0,187],[fuel_t,fuel_t]);
hold on;

c=fuel(196:1576,1);
d=fuel(196:1576,2);
for i=1:(1576-196)
    c(i)=c(i)-26.6004;
end   
plot(c,d,'-r');
% plot([0,187],[fuel_t,fuel_t]);
hold on;

e=rapid(76:1518,1);
f=rapid(76:1518,2);
for i=1:(1518-76)
    e(i)=e(i)-10.8362;
end   
plot(e,f,'-b');
% plot([0,187],[fuel_t,fuel_t]);
hold on;
