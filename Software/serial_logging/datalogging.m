% change the serial COM port number here
s = serialport("COM5", 115200);

% use the STOP button to exit the while loop
while true
    data_line = readline(s) % also print to the command window
    writelines(data_line,"log7.txt",WriteMode="append"); % save to text file
end


%%
% close the serial port by deleting the object
delete(s)