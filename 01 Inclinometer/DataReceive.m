if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

clear all;
close all;
%instrreset;
%serialportlist

disp('Press Ctrl+C to stop collecting data!')
% Baud rate:4800, 9600 (default), 19200 38400, 57600, 115200, 230400, stop

s = serial('COM6','baudrate', 9600) ;fopen(s) ;            % Please replace COM44 with the COM port recognized by the PC, and change the baud rate 9600 to the baud rate corresponding to the sensor
f = 20;         %DataFrequce
t = 0;
cnt = 1;
aa = [0 0 0];   % Acceleration X, Y, Z axis
ww = [0 0 0];   % Angular velocity X, Y, Z axis
AA = [0 0 0];   % Angle X, Y, Z axis
tt = 0;
a = [0 0 0]';
w = [0 0 0]';
A = [0 0 0]';

while(1)
    Head = fread(s,2,'uint8');                              % Getting serial data, the S file has been mentioned above
    if (Head(1)~=uint8(85))                                 % If the first data of the serial is not equal to 85 (0x55), it proves that it isn't conform to the protocol and haven't perform data analysis
        continue;
    end
    Head(2)
    switch(Head(2))                                         % Getting the second data of the serial
        case 81                                             % 81(0x51): Acceleration data packet
            a = fread(s,3,'int16')/32768*16;                % Getting three 16bit acceleration data, please refer to the protocol
        case 82                                             % 82 (0x52): Angular velocity data packet
            w = fread(s,3,'int16')/32768*2000;              % Getting three 16bit angular velocity data, please refer to the protocol
        case 83                                             % 83 (0x53): Angular data packet
            A = fread(s,3,'int16')/32768*180;               % Getting three 16bit angle data, please refer to the protocol.
            aa = [aa;a'];
            ww = [ww;w'];
            AA = [AA;A'];
            tt = [tt;t];
            
            %%%% Added code from here %%%%
            % degree to radian
            roll = deg2rad(A(1));
            pitch = deg2rad(A(2));
            yaw = deg2rad(A(3));

            % ZYX rotation matrix
            Rz = [cos(yaw) -sin(yaw) 0;
                  sin(yaw) cos(yaw) 0;
                  0 0 1];
            Ry = [cos(pitch) 0 sin(pitch);
                  0 1 0;
                  -sin(pitch) 0 cos(pitch)];
            Rx = [1 0 0;
                  0 cos(roll) -sin(roll);
                  0 sin(roll) cos(roll)];
            R = Rz * Ry * Rx;

            % roll is the inclination angle with respect to Y axis: downside(-), upside(+)
            Y_angle = atan2(R(3,2), R(3,3));

            % pitch is the inclination angle with respect to X axis: downside(-), upside(+)
            X_angle = -asin(-R(3,1));

            % radian to degree
            X_tilt = rad2deg(X_angle);
            Y_tilt = rad2deg(Y_angle);

            % print the result
            fprintf('X tilt: %.2fбу\n', X_tilt);
            fprintf('Y tilt: %.2fбу\n', Y_tilt);

            % print the result as a signed integer (rounded)
            fprintf('X tilt(rounded): %dбу\n', round(X_tilt));
            fprintf('Y tilt(rounded): %dбу\n', round(Y_tilt));

            % Previous code: plot the graph(Temporarily paused)
            % subplot(3,1,1);plot(tt,aa);title(['Acceleration = ' num2str(a') 'm2/s']);ylabel('m2/s');
            % subplot(3,1,2);plot(tt,ww);title(['Gyro = ' num2str(w') 'бу/s']);ylabel('бу/s');
            % subplot(3,1,3);plot(tt,AA);title(['Angle = ' num2str(A') 'бу']);ylabel('бу');              
            cnt = 0;
            % drawnow;
            
            if (size(aa,1)>5*f)                             % Clear history data
                aa = aa(f:5*f,:);
                ww = ww(f:5*f,:);
                AA = AA(f:5*f,:);
                tt = tt(f:5*f,:);
            end
            t = t + 0.1;                                    % The data default is 10Hz, which is 0.1s. If you change the output rate of the product, please change 0.1 to other output rates
    end
            End = fread(s,3,'uint8');
end

fclose(s);