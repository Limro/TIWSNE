close all
Kchoise = {R, F }
res = [];
for i = 1:2
    
K = Kchoise{i};

for l = 1:length(K)
    T = csvread(K{l,1},2,0);
    data = T(K{l,2}:K{l,3},2);
       
    
    data = ((2.7-data).^2)./10;
    data = filter(ones(1,100)*(1/100),1,data);
    
    O = csvread(R{l,1},2,0);
    figure, hold on, plot(T(:,1),T(:,2), 'b'),  plot(O(:,1),O(:,2), 'g');
    title('Shunt voltage (10 ohm)')
    ylabel('Voltage [V]');
    xlabel('Time [S]');
    g = cell(1,2);
    g{2}='Send'; g{1}='Recive';
    legend(g);
    
    
    s = mean(data); 
    
    Tj = T(K{l,2}:K{l,3},1);
    Tn = Tj + ones(length(Tj),1)*abs(min(Tj));
    dT = Tn(length(Tn))-Tn(1);
   
    sums(l,(i-1)*2+2) = s;
    
    sums(l,(i-1)*2+1) = s*dT;
    res(l,i) = s*dT;
end


end


for i = 1:4
    tres(i,1) = mean(res(((i-1)*3)+1:i*3)',1);
    tres(i,2) = mean(res(((i-1)*3)+1:i*3),2);
end

figure,
l = cell(1,2);
l{1}='Send'; l{2}='Recive';   

res = tres;

ax1 = subplot(2,1,1);
h = bar(ax1,res)
set(gca,'XTickLabel',{'8bit -> byte', '7 bit -> byte','6 bit -> byte','4 bit -> byte'})
colormap(summer(2));
grid on
legend(h,l);
title('TelosB Energy Usage')
ylabel('Energy [J]');

ax2 = subplot(2,1,2);
h = bar(ax2,res,'stacked')
colormap(summer(2));
grid on
legend(h,l);
ylabel('Energy [J]');
set(gca,'XTickLabel',{'8bit -> byte', '7 bit -> byte','6 bit -> byte','4 bit -> byte'})
