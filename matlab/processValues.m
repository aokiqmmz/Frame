filename = "MyValues.log";
dataOriTable = readtable(filename);

dataOri = table2array(dataOriTable).';

x = dataOri(1, :);
y = dataOri(2, :);

% sdisp(x)
% disp(y)

p = polyfit(x, y, 1);

disp(p(1))
disp(p(2))

x_dense = linspace(min(x), max(x), 100);
y_dense = polyval(p, x_dense);

scatter(x, y);
hold on;
plot(x_dense, y_dense);


