text = fileread("log5.txt");

% Input text to be parsed
%text = 'RX status: 0\n\nLatitude: 5240.153809\n\nLongitude: 131.434799\n\nDatetime: 123320.000000\n\nRSSI: -110\n\nSNR: 9\n\nRX status: 0\n\nTemperature: 34.459999\n\nHumidity: 32.627930\n\nPressure: 1006.085571\n\nRSSI: -105\n\nSNR: 8\n\nRX status: 0\n\nLatitude: 5240.153809\n\nLongitude: 131.434799\n\nDatetime: 123320.000000\n\nRSSI: -111\n\nSNR: 9\n\nRX status: 0\n\nTemperature: 34.459999\n\nHumidity: 32.627930\n\nPressure: 1006.083923\n\nRSSI: -108\n\nSNR: 9\n\nRX status: 0\n\nLatitude: 5240.153809\n\nLongitude: 131.434799\n\nDatetime: 123320.000000\n\nRSSI: -114\n\nSNR: 9\n\nRX status: 0\n\nTemperature: 34.470001\n\nHumidity: 32.627930\n\nPressure: 1006.086304\n\nRSSI: -108\n\nSNR: 9\n\nRX status: 0\n\nLatitude: 5240.153809\n\nLongitude: 131.434799\n\nDatetime: 123320.000000\n\nRSSI: -114\n\nSNR: 9';

% Split the text into individual lines
lines = strsplit(text, '\n');

% Initialize variables
data = [];
currentEntry = [];

latBegin = 52.669230150000000;
longBegin = -1.523913320000000;
wgs84 = wgs84Ellipsoid("m");

% Parse each line of the text
for i = 0:floor(numel(lines)/6)-1
    latitude = sscanf(lines{i*6+1}, 'Latitude: %s');
    currentEntry.Latitude = round(str2double(latitude(1:2))+str2double(latitude(3:11))/60,8);
    longitude = sscanf(lines{i*6+2}, 'Longitude: %s');
    currentEntry.Longitude = -round(str2double(longitude(1))+str2double(longitude(2:10))/60,8);
    currentEntry.Distance = distance(latBegin,longBegin,currentEntry.Latitude,currentEntry.Longitude,wgs84);
    datetime = sscanf(lines{i*6+3}, 'Datetime: %f');
    currentEntry.Datetime = datetime;
    %rssi = sscanf(lines{i*10+4}, 'RSSI: %d');
    %currentEntry.RSSI = rssi;
    %snr = sscanf(lines{i*10+5}, 'SNR: %d');
    %currentEntry.SNR = snr;
    temperature = sscanf(lines{i*6+4}, 'Temperature: %f');
    currentEntry.temperature = temperature;
    humidity = sscanf(lines{i*6+5}, 'Humidity: %f');
    currentEntry.humidity = humidity;
    pressure = sscanf(lines{i*6+6}, 'Pressure: %f');
    currentEntry.pressure = pressure*100;
    if ~isempty(currentEntry)
        data = [data; currentEntry];
        currentEntry = [];
    end
end

% Add the last entry to the data
if ~isempty(currentEntry)
    data = [data; currentEntry];
end

% Convert the parsed data to a table
tableData = struct2table(data);

% Time axis
timeaxis = (1:length(tableData.pressure))*0.2;

figure(1)
plot(timeaxis, tableData.pressure, 'LineWidth',2)
grid on
grid minor
title("Pressure [Pa]")

figure(2)
plot(timeaxis, tableData.humidity,'LineWidth',2)
grid on
grid minor
title("Humidity [%RH]")

figure(3)
plot(timeaxis, tableData.temperature,'LineWidth',2)
grid on
grid minor
title("Temperature [deg]")

figure(4)
plot(timeaxis, tableData.Distance, 'LineWidth',2)
grid on
grid minor
title("Distance [m]")

fig5 = figure(5)
geobasemap satellite
geoplot(tableData.Latitude,tableData.Longitude,"r","LineWidth",3);
set(fig5, 'Position', [1000, 1000, width, height]);
set(gca, 'FontSize', 24);

% Height
fig6 = figure(6)
width = 800;  % Specify the figure width in pixels
height = 600; % Specify the figure height in pixels
set(fig6, 'Position', [1000, 1000, width, height]);

T_0 = 295.8;
R_d = 287.059;
P_0 = 100603;
g = 9.80665;
plot(timeaxis, T_0/0.0065*(1-exp(0.0065*R_d/g*log(tableData.pressure/P_0))), 'LineWidth',5);
ylabel('Height from ground level [m]', 'FontSize', 30); 
xlabel('Time [s]', 'FontSize', 30); 
grid on
grid minor
set(gca, 'FontSize', 30);

% Display the table
%disp(tableData);