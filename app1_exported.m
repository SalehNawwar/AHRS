classdef app1_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                matlab.ui.Figure
        Panel                   matlab.ui.container.Panel
        BaudeRateDropDown       matlab.ui.control.DropDown
        BaudeRateDropDownLabel  matlab.ui.control.Label
        StatusLamp              matlab.ui.control.Label
        DisconnectButton        matlab.ui.control.Button
        ConnectButton           matlab.ui.control.Button
        COMPortDropDown         matlab.ui.control.DropDown
        SerialPortLabel         matlab.ui.control.Label
        TabGroup                matlab.ui.container.TabGroup
        Tab                     matlab.ui.container.Tab
        SensorsPanel            matlab.ui.container.Panel
        mzCheckBox              matlab.ui.control.CheckBox
        myCheckBox              matlab.ui.control.CheckBox
        mxCheckBox              matlab.ui.control.CheckBox
        gzCheckBox              matlab.ui.control.CheckBox
        gyCheckBox              matlab.ui.control.CheckBox
        gxCheckBox              matlab.ui.control.CheckBox
        azCheckBox              matlab.ui.control.CheckBox
        ayCheckBox              matlab.ui.control.CheckBox
        axCheckBox              matlab.ui.control.CheckBox
        UIAxes                  matlab.ui.control.UIAxes
        Tab2                    matlab.ui.container.Tab
        StatePanel              matlab.ui.container.Panel
        magzCheckBox            matlab.ui.control.CheckBox
        magyCheckBox            matlab.ui.control.CheckBox
        magxCheckBox            matlab.ui.control.CheckBox
        gravzCheckBox           matlab.ui.control.CheckBox
        gravyCheckBox           matlab.ui.control.CheckBox
        gravxCheckBox           matlab.ui.control.CheckBox
        bgzCheckBox             matlab.ui.control.CheckBox
        bgyCheckBox             matlab.ui.control.CheckBox
        bgxCheckBox             matlab.ui.control.CheckBox
        yawCheckBox             matlab.ui.control.CheckBox
        pitchCheckBox           matlab.ui.control.CheckBox
        rollCheckBox            matlab.ui.control.CheckBox
        UIAxesState             matlab.ui.control.UIAxes
        Tab3                    matlab.ui.container.Tab
        SonsorsLabel            matlab.ui.control.Label
        StateLabel              matlab.ui.control.Label
    end

    
    properties (Access = public)
        arduinoObj        % To hold the serialport object
        plotHandles=struct();        % Handle for the animatedline
        dataBuffer        % Buffer to store recent data for calculations
        LastCounter = 1;
        point = "Text";
        Counter =1;      % Simple counter for the x-axis
        BufferSize = 100000; % How many points to store for calculations
        flags = struct();
        data = struct();
        Frame
        viewLength = 30;% in secs
        Timer
        UITimer
        plotsNames = ["ax","ay","az","gx","gy","gz","mx","my","mz","roll","pitch","yaw","bgx","bgy","bgz","gravx","gravy","gravz","magx","magy","magz"];
        
        plotsColors = ['b','r','g','b','r','g','b','r','g','b','r','g','b','r','g','b','r','g','b','r','g'];
        ekf = extendedKalmanFilter(@transitionfun, @measurementfun_g);
        X = [1,0,0,0,0,0,0,0,0,0,0,0,0];
        g_ref
        m_ref
        bm = [-0.065,0.377,-0.215];

    end
    
    methods (Access = private)
        
        function dataAvailableCallback(app)
            stop(app.Timer);
            if app.arduinoObj.NumBytesAvailable == 0
                start(app.Timer);
                return;
            end
            % Read the latest line of data
            
            while app.arduinoObj.NumBytesAvailable>43
            
                app.data.count(app.Counter) = double(read(app.arduinoObj,1,'uint32'));
                app.data.t(app.Counter) = double(read(app.arduinoObj,1,'uint32'));
%                 disp("sensors");
                for index = 1:9
                    name = app.plotsNames(index);
                    app.data.(name)(app.Counter) = read(app.arduinoObj,1,'single');
%                     disp(app.data.(name)(app.Counter))
%                     disp(name);
                end
                app.Counter = app.Counter+1;
            end

%             app.data.a(app.Counter,:) = read(app.arduinoObj,3,'single');
%             app.g(app.Counter,:) = read(app.arduinoObj,3,'single');
%             app.m(app.Counter,:) = read(app.arduinoObj,3,'single');

            
            % --- REAL-TIME PLOTTING ---
            while app.LastCounter<app.Counter
%                 for index = 1:length(app.plotsNames)
%                     
%                     addpoints(app.plotHandles.(app.plotsNames(index)), app.data.t(app.LastCounter), app.data.(app.plotsNames(index))(app.LastCounter));
%                     
%                 end
                
                dt = 0;
                if app.LastCounter ~= 1
                    dt = (app.data.t(app.LastCounter)-app.data.t(app.LastCounter-1))/1000;
                end
                G = zeros(13, 3);
                q_mult = q_mult_matrix(app.ekf.State(1:4));
                G(1:4, :) = 0.5 * q_mult(:, 2:end);
                    Q_w = eye(3)*(deg2rad(1)^2);
                Q = G * Q_w * G' * dt;
                app.ekf.ProcessNoise = Q;
                
                app.ekf.predict([dt, ...
                    app.data.gx(app.LastCounter), ...
                    app.data.gy(app.LastCounter), ...
                    -app.data.gz(app.LastCounter)]);

                if app.LastCounter<200
                    if app.LastCounter==1
                        app.g_ref = [app.data.ax(1),app.data.ay(1),-app.data.az(1)];
                        app.m_ref = [ -(app.data.mx(1)-app.bm(1)),app.data.my(1)-app.bm(2),-(app.data.mz(1)-app.bm(3)) ];
                    else
                        app.g_ref = (app.g_ref*(app.LastCounter-1)+...
                            [app.data.ax(app.LastCounter),app.data.ay(app.LastCounter),-app.data.az(app.LastCounter)])/(app.LastCounter);
                        app.m_ref = (app.m_ref*(app.LastCounter-1)+...
                            [-(app.data.mx(app.LastCounter)-app.bm(1)),app.data.my(app.LastCounter)-app.bm(2),-(app.data.mz(app.LastCounter)-app.bm(3))])/(app.LastCounter);
                    end
                
                elseif app.LastCounter==200
                    disp([app.g_ref,app.m_ref]);
                    app.ekf.State(8:13) = [app.g_ref,app.m_ref];
                elseif app.LastCounter<app.Counter-2 && app.LastCounter>2
                    app.ekf.correct([ ...
                        median(app.data.ax(app.LastCounter-2:app.LastCounter+2)), ...
                        median(app.data.ay(app.LastCounter-2:app.LastCounter+2)), ...
                        -median(app.data.az(app.LastCounter-2:app.LastCounter+2)), ...
                        -median(app.data.mx(app.LastCounter-2:app.LastCounter+2)-app.bm(1)), ...
                        median(app.data.my(app.LastCounter-2:app.LastCounter+2))-app.bm(2), ...
                        -median(app.data.mz(app.LastCounter-2:app.LastCounter+2)-app.bm(3))]);
                end
                
                app.X(app.LastCounter,:) = app.ekf.State; 
                eulers = rad2deg(quat2eul(app.ekf.State(1:4)));
                app.data.yaw(app.LastCounter) = eulers(1);
                app.data.pitch(app.LastCounter) = eulers(2);
                app.data.roll(app.LastCounter) = eulers(3);
                
                for index = 13:21
                    app.data.(app.plotsNames(index))(app.LastCounter) = app.ekf.State(index-8);
                end

                app.LastCounter = app.LastCounter + 1;
            end
                    
            start(app.Timer);
        end
        
        function updateUI(app)
            
            stop(app.UITimer);
            idx = 1;
            disp([app.LastCounter,app.Counter]);
            
            if app.LastCounter > 1
                idx = app.LastCounter-1;
            end
            for index = 1:length(app.plotsNames)
                
                addpoints(app.plotHandles.(app.plotsNames(index)), app.data.t(idx)/1000, app.data.(app.plotsNames(index))(idx));
                
            end
            if app.data.t(idx)/1000 > app.viewLength
                xlim(app.UIAxes,[app.data.t(idx)/1000-app.viewLength,app.data.t(idx)/1000]);
                xlim(app.UIAxesState,[app.data.t(idx)/1000-app.viewLength,app.data.t(idx)/1000]);
            else
                xlim(app.UIAxes,[0,app.viewLength]);
                xlim(app.UIAxesState,[0,app.viewLength]);
            end
            drawnow limitrate;

            app.StateLabel.Text = num2str( app.ekf.State);
            app.SonsorsLabel.Text = num2str([app.data.count(idx),app.data.t(idx), ...
                app.data.ax(idx),app.data.ay(idx),app.data.az(idx), ...
                app.data.gx(idx),app.data.gy(idx),app.data.gz(idx), ...
                app.data.mx(idx),app.data.my(idx),app.data.mz(idx)]);
            start(app.UITimer);
        end
        
        function InitializeKalman(app)
            app.X = zeros(app.BufferSize,13);
            app.X(1,1) = 1;
            app.X(1,8:10) = [0,0,-1];
%             app.X(1,14:16) = [-0.065,0.377,-0.215]; % m bias
            app.ekf.State = app.X(1,:);
            app.ekf.StateCovariance = diag([0.5,0.5,0.5,0.5, ...
                deg2rad(10),deg2rad(10),deg2rad(10), ...
                0.2,0.2,0.2, ...
                1,1,1].^2);
            app.ekf.MeasurementNoise = zeros(6);
            app.ekf.MeasurementNoise(1:3,1:3) = eye(3)*0.1^2; % acc measure noise
            app.ekf.MeasurementNoise(4:6,4:6) = eye(3)*0.05^2; % mag measure noise
            app.Counter = 1;
%             app.Frame
            app.LastCounter = 1;
            app.g_ref = [0,0,0];
            app.m_ref = [0,0,0];
        end
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            % Find all available serial ports and populate the dropdown
            app.COMPortDropDown.Items = serialportlist("available");
            if ~isempty(app.COMPortDropDown.Items)
                app.COMPortDropDown.Value = app.COMPortDropDown.Items{1};
            end

            app.data.t = zeros(1,app.BufferSize);
            app.data.count = zeros(1,app.BufferSize);
            app.Frame = uilabel(app.UIFigure);
            app.Frame.(app.point) = 'Saleh Nawwar & Younes Ahmad';
            app.Frame.FontSize = 14;
            app.Frame.Position = [250,100,400,20];
            for name = app.plotsNames
                app.data.(name) = zeros(1,app.BufferSize);
                app.flags.(name) = false;
            end
            
            
            % Configure initial GUI state
            app.DisconnectButton.Enable = 'off';
            app.StatusLamp.FontColor = 'red';
            app.StatusLamp.Text = 'Disconnected';
            app.Counter = 1;
            app.dataBuffer = [];

        
            % Setup the plot
            title(app.UIAxes, "Sensor Data");
            xlabel(app.UIAxes, "Time (s)");
            ylabel(app.UIAxes, "Sensor Value");
            grid(app.UIAxes, "on");
            xlim(app.UIAxes,[0,30]);
            title(app.UIAxesState, "State");
            xlabel(app.UIAxesState, "Time (s)");
            ylabel(app.UIAxesState, "State Value");
            grid(app.UIAxesState, "on");
            
            %timer setup
            app.Timer = timer;
            app.UITimer = timer;
            app.Timer.StartDelay = 0.02;
            app.UITimer.StartDelay = 0.3;
            app.Timer.TimerFcn = @(~,~)dataAvailableCallback(app);
            app.UITimer.TimerFcn = @(~,~)updateUI(app);
        end

        % Button pushed function: ConnectButton
        function ConnectButtonPushed(app, event)
            try
                InitializeKalman(app);

                % Get selected COM port and set baud rate
                comPort = app.COMPortDropDown.Value;
                baudRate = str2double(app.BaudeRateDropDown.Value); % Must match Arduino's Serial.begin()
                
                disp(baudRate);
                % THE NON-BLOCKING MAGIC:
                % Assign a callback function to be executed when time tics
                
                % Update GUI state
                app.ConnectButton.Enable = 'off';
                app.DisconnectButton.Enable = 'on';
                app.StatusLamp.FontColor = 'green';
                app.StatusLamp.Text = 'Connected';
                app.COMPortDropDown.Enable = 'off';
        
                % Initialize the plot
                cla(app.UIAxes); % Clear the axes
                cla(app.UIAxesState);
                for index = 1:length(app.plotsNames)
                    activeAxes = app.UIAxes;
                    if index>9
                        activeAxes = app.UIAxesState;
                    end
                    app.plotHandles.(app.plotsNames(index)) = animatedline(activeAxes, 'Color', app.plotsColors(index), 'Marker', '.');
                    app.plotHandles.(app.plotsNames(index)).Visible = app.flags.(app.plotsNames(index));
                end
                title(app.UIAxes, "Sensor Data (Connected)");
                
                % Create and configure the serialport object
                app.arduinoObj = serialport(comPort, baudRate,'ByteOrder','little-endian');
                flush(app.arduinoObj);
                while 1
                    mark = read(app.arduinoObj,1,'uint16');
    
                    if mark==200
                        break
                    end
                end

                start(app.Timer);
                start(app.UITimer);

            catch ME
                % Update GUI state
                app.ConnectButton.Enable = 'on';
                app.DisconnectButton.Enable = 'off';
                app.StatusLamp.FontColor = 'red';
                app.StatusLamp.Text = 'Disconnected';
                app.COMPortDropDown.Enable = 'on';
                title(app.UIAxes, "Sensor Data (Disconnected)");
                % If connection fails, show an error
                uialert(app.UIFigure, ME.message, "Connection Error");
            end
            
        end

        % Button pushed function: DisconnectButton
        function DisconnectButtonPushed(app, event)
            % Clear the serial object to close the connection
            stop(app.Timer);
            stop(app.UITimer);
            delete(app.arduinoObj);
            clear app.arduinoObj;
            
            % Update GUI state
            app.ConnectButton.Enable = 'on';
            app.DisconnectButton.Enable = 'off';
            app.StatusLamp.FontColor = 'red';
            app.StatusLamp.Text = 'Disconnected';
            app.COMPortDropDown.Enable = 'on';
            title(app.UIAxes, "Sensor Data (Disconnected)");
            
            % Reset data
            app.Counter = 1;
            app.LastCounter = 1;
            app.dataBuffer = [];
            
        end

        % Close request function: UIFigure
        function UIFigureCloseRequest(app, event)
            % Clear the serial object to close the connection
            
            delete(app.Timer);
            delete(app.UITimer);
            clear app.arduinoObj;
            
            % Delete the figure
            delete(app);
        end

        % Value changed function: axCheckBox, ayCheckBox, azCheckBox, 
        % ...and 18 other components
        function axCheckBoxValueChanged(app, event)
         
            app.flags.(event.Source.Text) = event.Source.Value;
            app.plotHandles.(event.Source.Text).Visible = event.Source.Value;
            
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 991 568];
            app.UIFigure.Name = 'MATLAB App';
            app.UIFigure.CloseRequestFcn = createCallbackFcn(app, @UIFigureCloseRequest, true);

            % Create TabGroup
            app.TabGroup = uitabgroup(app.UIFigure);
            app.TabGroup.Position = [1 116 992 453];

            % Create Tab
            app.Tab = uitab(app.TabGroup);
            app.Tab.Title = 'Tab';

            % Create UIAxes
            app.UIAxes = uiaxes(app.Tab);
            title(app.UIAxes, 'Title')
            xlabel(app.UIAxes, 'X')
            ylabel(app.UIAxes, 'Y')
            zlabel(app.UIAxes, 'Z')
            app.UIAxes.Position = [1 1 881 428];

            % Create SensorsPanel
            app.SensorsPanel = uipanel(app.Tab);
            app.SensorsPanel.Title = 'Sensors Panel';
            app.SensorsPanel.Position = [891 16 100 413];

            % Create axCheckBox
            app.axCheckBox = uicheckbox(app.SensorsPanel);
            app.axCheckBox.ValueChangedFcn = createCallbackFcn(app, @axCheckBoxValueChanged, true);
            app.axCheckBox.Text = 'ax';
            app.axCheckBox.Position = [34 354 35 22];

            % Create ayCheckBox
            app.ayCheckBox = uicheckbox(app.SensorsPanel);
            app.ayCheckBox.ValueChangedFcn = createCallbackFcn(app, @axCheckBoxValueChanged, true);
            app.ayCheckBox.Text = 'ay';
            app.ayCheckBox.Position = [34 314 35 22];

            % Create azCheckBox
            app.azCheckBox = uicheckbox(app.SensorsPanel);
            app.azCheckBox.ValueChangedFcn = createCallbackFcn(app, @axCheckBoxValueChanged, true);
            app.azCheckBox.Text = 'az';
            app.azCheckBox.Position = [34 274 35 22];

            % Create gxCheckBox
            app.gxCheckBox = uicheckbox(app.SensorsPanel);
            app.gxCheckBox.ValueChangedFcn = createCallbackFcn(app, @axCheckBoxValueChanged, true);
            app.gxCheckBox.Text = 'gx';
            app.gxCheckBox.Position = [34 234 35 22];

            % Create gyCheckBox
            app.gyCheckBox = uicheckbox(app.SensorsPanel);
            app.gyCheckBox.ValueChangedFcn = createCallbackFcn(app, @axCheckBoxValueChanged, true);
            app.gyCheckBox.Text = 'gy';
            app.gyCheckBox.Position = [34 194 35 22];

            % Create gzCheckBox
            app.gzCheckBox = uicheckbox(app.SensorsPanel);
            app.gzCheckBox.ValueChangedFcn = createCallbackFcn(app, @axCheckBoxValueChanged, true);
            app.gzCheckBox.Text = 'gz';
            app.gzCheckBox.Position = [34 154 35 22];

            % Create mxCheckBox
            app.mxCheckBox = uicheckbox(app.SensorsPanel);
            app.mxCheckBox.ValueChangedFcn = createCallbackFcn(app, @axCheckBoxValueChanged, true);
            app.mxCheckBox.Text = 'mx';
            app.mxCheckBox.Position = [34 115 38 22];

            % Create myCheckBox
            app.myCheckBox = uicheckbox(app.SensorsPanel);
            app.myCheckBox.ValueChangedFcn = createCallbackFcn(app, @axCheckBoxValueChanged, true);
            app.myCheckBox.Text = 'my';
            app.myCheckBox.Position = [34 76 38 22];

            % Create mzCheckBox
            app.mzCheckBox = uicheckbox(app.SensorsPanel);
            app.mzCheckBox.ValueChangedFcn = createCallbackFcn(app, @axCheckBoxValueChanged, true);
            app.mzCheckBox.Text = 'mz';
            app.mzCheckBox.Position = [34 37 38 22];

            % Create Tab2
            app.Tab2 = uitab(app.TabGroup);
            app.Tab2.Title = 'Tab2';

            % Create UIAxesState
            app.UIAxesState = uiaxes(app.Tab2);
            title(app.UIAxesState, 'Title')
            xlabel(app.UIAxesState, 'X')
            ylabel(app.UIAxesState, 'Y')
            zlabel(app.UIAxesState, 'Z')
            app.UIAxesState.Position = [1 1 863 428];

            % Create StatePanel
            app.StatePanel = uipanel(app.Tab2);
            app.StatePanel.Title = 'State Panel';
            app.StatePanel.Position = [881 15 110 413];

            % Create rollCheckBox
            app.rollCheckBox = uicheckbox(app.StatePanel);
            app.rollCheckBox.ValueChangedFcn = createCallbackFcn(app, @axCheckBoxValueChanged, true);
            app.rollCheckBox.Text = 'roll';
            app.rollCheckBox.Position = [30 365 38 22];

            % Create pitchCheckBox
            app.pitchCheckBox = uicheckbox(app.StatePanel);
            app.pitchCheckBox.ValueChangedFcn = createCallbackFcn(app, @axCheckBoxValueChanged, true);
            app.pitchCheckBox.Text = 'pitch';
            app.pitchCheckBox.Position = [30 333 47 22];

            % Create yawCheckBox
            app.yawCheckBox = uicheckbox(app.StatePanel);
            app.yawCheckBox.ValueChangedFcn = createCallbackFcn(app, @axCheckBoxValueChanged, true);
            app.yawCheckBox.Text = 'yaw';
            app.yawCheckBox.Position = [30 301 43 22];

            % Create bgxCheckBox
            app.bgxCheckBox = uicheckbox(app.StatePanel);
            app.bgxCheckBox.ValueChangedFcn = createCallbackFcn(app, @axCheckBoxValueChanged, true);
            app.bgxCheckBox.Text = 'bgx';
            app.bgxCheckBox.Position = [30 269 41 22];

            % Create bgyCheckBox
            app.bgyCheckBox = uicheckbox(app.StatePanel);
            app.bgyCheckBox.ValueChangedFcn = createCallbackFcn(app, @axCheckBoxValueChanged, true);
            app.bgyCheckBox.Text = 'bgy';
            app.bgyCheckBox.Position = [30 237 41 22];

            % Create bgzCheckBox
            app.bgzCheckBox = uicheckbox(app.StatePanel);
            app.bgzCheckBox.ValueChangedFcn = createCallbackFcn(app, @axCheckBoxValueChanged, true);
            app.bgzCheckBox.Text = 'bgz';
            app.bgzCheckBox.Position = [30 205 41 22];

            % Create gravxCheckBox
            app.gravxCheckBox = uicheckbox(app.StatePanel);
            app.gravxCheckBox.ValueChangedFcn = createCallbackFcn(app, @axCheckBoxValueChanged, true);
            app.gravxCheckBox.Text = 'gravx';
            app.gravxCheckBox.Position = [30 173 51 22];

            % Create gravyCheckBox
            app.gravyCheckBox = uicheckbox(app.StatePanel);
            app.gravyCheckBox.ValueChangedFcn = createCallbackFcn(app, @axCheckBoxValueChanged, true);
            app.gravyCheckBox.Text = 'gravy';
            app.gravyCheckBox.Position = [30 141 51 22];

            % Create gravzCheckBox
            app.gravzCheckBox = uicheckbox(app.StatePanel);
            app.gravzCheckBox.ValueChangedFcn = createCallbackFcn(app, @axCheckBoxValueChanged, true);
            app.gravzCheckBox.Text = 'gravz';
            app.gravzCheckBox.Position = [30 109 51 22];

            % Create magxCheckBox
            app.magxCheckBox = uicheckbox(app.StatePanel);
            app.magxCheckBox.ValueChangedFcn = createCallbackFcn(app, @axCheckBoxValueChanged, true);
            app.magxCheckBox.Text = 'magx';
            app.magxCheckBox.Position = [30 77 51 22];

            % Create magyCheckBox
            app.magyCheckBox = uicheckbox(app.StatePanel);
            app.magyCheckBox.ValueChangedFcn = createCallbackFcn(app, @axCheckBoxValueChanged, true);
            app.magyCheckBox.Text = 'magy';
            app.magyCheckBox.Position = [30 45 51 22];

            % Create magzCheckBox
            app.magzCheckBox = uicheckbox(app.StatePanel);
            app.magzCheckBox.ValueChangedFcn = createCallbackFcn(app, @axCheckBoxValueChanged, true);
            app.magzCheckBox.Text = 'magz';
            app.magzCheckBox.Position = [30 13 51 22];

            % Create Tab3
            app.Tab3 = uitab(app.TabGroup);
            app.Tab3.Title = 'Tab3';

            % Create StateLabel
            app.StateLabel = uilabel(app.Tab3);
            app.StateLabel.Position = [43 380 911 33];

            % Create SonsorsLabel
            app.SonsorsLabel = uilabel(app.Tab3);
            app.SonsorsLabel.Position = [43 298 911 33];

            % Create Panel
            app.Panel = uipanel(app.UIFigure);
            app.Panel.Title = 'Panel';
            app.Panel.Position = [14 11 948 88];

            % Create SerialPortLabel
            app.SerialPortLabel = uilabel(app.Panel);
            app.SerialPortLabel.HorizontalAlignment = 'right';
            app.SerialPortLabel.Position = [19 20 62 22];
            app.SerialPortLabel.Text = 'Serial Port';

            % Create COMPortDropDown
            app.COMPortDropDown = uidropdown(app.Panel);
            app.COMPortDropDown.Position = [96 20 100 22];

            % Create ConnectButton
            app.ConnectButton = uibutton(app.Panel, 'push');
            app.ConnectButton.ButtonPushedFcn = createCallbackFcn(app, @ConnectButtonPushed, true);
            app.ConnectButton.Position = [228 20 100 23];
            app.ConnectButton.Text = 'Connect';

            % Create DisconnectButton
            app.DisconnectButton = uibutton(app.Panel, 'push');
            app.DisconnectButton.ButtonPushedFcn = createCallbackFcn(app, @DisconnectButtonPushed, true);
            app.DisconnectButton.Position = [358 20 100 23];
            app.DisconnectButton.Text = 'Disconnect';

            % Create StatusLamp
            app.StatusLamp = uilabel(app.Panel);
            app.StatusLamp.Position = [496 20 144 22];
            app.StatusLamp.Text = 'Status';

            % Create BaudeRateDropDownLabel
            app.BaudeRateDropDownLabel = uilabel(app.Panel);
            app.BaudeRateDropDownLabel.HorizontalAlignment = 'right';
            app.BaudeRateDropDownLabel.Position = [714 20 69 22];
            app.BaudeRateDropDownLabel.Text = 'Baude Rate';

            % Create BaudeRateDropDown
            app.BaudeRateDropDown = uidropdown(app.Panel);
            app.BaudeRateDropDown.Items = {'9600', '115200'};
            app.BaudeRateDropDown.Position = [798 20 100 22];
            app.BaudeRateDropDown.Value = '115200';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = app1_exported

            runningApp = getRunningApp(app);

            % Check for running singleton app
            if isempty(runningApp)

                % Create UIFigure and components
                createComponents(app)

                % Register the app with App Designer
                registerApp(app, app.UIFigure)

                % Execute the startup function
                runStartupFcn(app, @startupFcn)
            else

                % Focus the running singleton app
                figure(runningApp.UIFigure)

                app = runningApp;
            end

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end