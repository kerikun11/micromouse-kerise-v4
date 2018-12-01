% v = csvread('volt.csv');
v = tblread('data.txt', '\t');
plot(v(:,8));
