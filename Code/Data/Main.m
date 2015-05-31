close all
Kchoise = {R, F }
res = [];
for i = 1:2
    
K = Kchoise{i};

for l = 1:length(K)
    T = csvread(K{l,1},2,0);
    plot(T(:,2))
    data = T(K{l,2}:K{l,3},2);
       
    
    data = (data.^2)./10;
    data = filter(ones(1,100)*(1/100),1,data);
    figure,
    hold on
    plot(data,'r');
    
    
    s = mean(data); 
    
    Tj = T(K{l,2}:K{l,3},1);
    Tn = Tj + ones(length(Tj),1)*abs(min(Tj));
    dT = Tn(length(Tn))-Tn(1);
   
    sums(l,2) = dT;
    
    sums(l,1) = s*dT;
    res(l,i) = s*dT;
end


end

figure,

l = cell(1,2);
l{1}='Send'; l{2}='Recive';   


ax1 = subplot(2,1,1);
h = bar(ax1,res)
set(gca,'XTickLabel',{'','8bit -> byte', '', '', '7 bit -> byte','','','6 bit -> byte','','','4 bit -> byte'})
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
set(gca,'XTickLabel',{'','8bit -> byte', '', '', '7 bit -> byte','','','6 bit -> byte','','','4 bit -> byte'})
