% the wheelchair is control in differential mode. see https://chess.eecs.berkeley.edu/eecs149/documentation/differentialDrive.pdf
% V5, add in N targets in a circle, and let the user to drive the wheelchair to reach
% each target.
% V6.1 corrected a bug on distance calculation.
% V6.2 correction a bug on continuous - discrete model problem in wheelchair_model
% v6.3 change from 3 class to 4 class, 'stop' is added. if no signal is
% comming in, the motion states will be remained.
% v6.6 using graphics timer, added joystick
% v6.7 target changed to yellow, added modes for only straight, left/right
% turning. Ability to change size of triangle and target
% v7 cleaned up code, added one more mode (mode 3) for turn then move
% straight (left/right), added border so wheelchair doesn't go off screen,
% created session_data for saving purposes
% v7_1 added feature of monkey needing to hold joystick at neutral position
% for 2 sec before start of each trial.
% v 7_2 added digital events to be sent to Plexon
% 'c'= Event1;'b'=Event2; 'a'=Event3
% v7_3 Modes 21,22, 31
% added juice reward in parameters file
% v7_4 animal needs to stop for x amount of time before reward
% v7_5 added comment logging functionality
% v7_6 added input interface support, incorporated try, catch structure
% v7_7 added mode 32 (randomize among 1, 21 and 22), tested setup on
% 25047
% v7_8 added mode 33, which is the original mode 32; changed mode 32 to
% repeat the same type of mode until succeeded
% v8_0 re-organized code to check for COM ports. Also changed
% joystick_detect, for limiting modes 21 and 22 to +-150
% degree range
% --
% v8_1 major changes to code (back-end). Other than reordering
% parameters_v8.txt, there is little change to user input method from v8_0.
% Incorporated feature to control via keyboard, for testing in the absence
% of arduino joystick (can be toggled on parameter txt)
% v8_2 trajectory saves x, y and theta; recalculated centroid
% v8_3 added new modes
% v8_4 events updated to indicate start and end of trial + reward
% v8_5 CORRECTED SPELLING OF FILENAME, added additional parameters
% 'use_guide_rails', 'narrow_guide', 'random_stratified'
% v8_6 Added a 'randomized time extension" which will vary the duration for
% each trial. (Set this parameter to 0 to disable the randomized trial
% durations); added a new timer (trajTiming, record time when coordinate and
% trajectory are saved; adjusted forward target to be closer distance;
% added "full" timing ("keeptiming") in addition to trial timing to
% session_data (as field TrialFullTiming)
% v8_7 Added fixation cross before start of trial. Smiley face for reward,
% red cross for no reward. Added sound for reward
% v8_8 Able to change size of target
% v8_9 Trial terminates immediately if overshoot
% v8_10 Trial terminates immediately if stops within green
% circle and then moves again.
% v8_11 Session ends automatically after a certain number of successful
% trials
% v8_12 Target changed from circle to square.
% v8_13 Target disappears and reappears
% Last modified: Rosa So, 12 March 2018


close all;
delete(instrfindall);
% clear all variabless (local and global), but preserve sessNum for reference
clearvars -except sessNum;
clearvars -global -except sessNum;
clear mex
clear functions
% clear import
% clear all
clc
load('wheel_model.mat','A_1','B_1','C_1','D_1');
Screen('Preference', 'SkipSyncTests', 1);

%% User Defined Settings

%Adds path
currentdir=pwd; %automatically gets current directory
cd(currentdir);
addpath (genpath('.\'))

% Debug Mode runs the program in windowed mode (small window, non-fullscreen)
debug_mode = false;
if debug_mode
    warning('Debug Mode is ON. Running program in windowed mode.')
end

% Uses a pentagon arrow for 'wheelchair' instead of triangle
useArrow = true;
% useArrow = false;

%% Internal code version tracking
codeVer = 8.12;

% Internal check of code version
% -- Do not change this check -- %
if codeVer < 8.1 || ~exist('codeVer','var')
    error('Changes to the code is not backward compatible. Please ensure that you are running the latest version of Wheelchair Simulation code (>=8.1)')
end
% -- Do not change this check -- %


%% Get references to keyboard keys
KbName('UnifyKeyNames');
global upKey downKey leftKey rightKey escapeKey kKey trialMode
upKey = KbName('UpArrow');
downKey = KbName('DownArrow');
leftKey = KbName('LeftArrow');
rightKey = KbName('RightArrow');
escapeKey = KbName('ESCAPE');
kKey= KbName('k');

%% Session setup (Request inputs + Reading parameters)
try
    % Open a question dialog with three options
    choice = questdlg('Please select the subject name', ...
        'Monkey Name', ...
        'Datou','Test','Others...','Datou');
    % Handle response
    switch choice
        case 'Datou'
            participant='Datou';
        case 'Test'
            participant='Test';
        % If "Others" is selected, an input dialog appears to ask for
        % subject's name
        case 'Others...'
            strinput = inputdlg('Enter Monkey Name:',...
                 'Monkey Name', [1 50]);
            participant = strinput{:};
    end

    % Open an input dialog to enter session number.
    % Default value: Previous session number + 1
    if ~exist('sessNum','var')
        sessNum = 0;
    end
    nextsessNum = sessNum + 1;
    numinput = inputdlg('Enter Session Number:',...
                 'Session Number', [1 50], {num2str(nextsessNum)});
    sessNum = str2double(numinput{:});

    % Open an input dialog to enter number of trials.
    % Default value: 20
    defaultNumTrials = 1000;
    num2input = inputdlg('Enter Number of Trials:',...
                 'No. of Trials', [1 50], {num2str(defaultNumTrials)});
    numTrials = str2double(num2input{:});

    defaultSucNumTrials = 60;
    num2input = inputdlg('Enter Number of Successful Trials:',...
                 'No. of Successful Trials', [1 50], {num2str(defaultSucNumTrials)});
    numSucTrials = str2double(num2input{:});

    % Get current date and time
    currentTime=clock;
    dateString=[num2str(currentTime(1)-2000,'%02d') num2str(currentTime(2),'%02d') num2str(currentTime(3),'%02d')];
    timeStartString=[num2str(currentTime(4),'%02d') ':' num2str(currentTime(5),'%02d') ':' num2str(floor(currentTime(6)),'%02d')];
    sav=strcat('data/',participant,'_',dateString,'_S',num2str(sessNum,'%02d'));
    % Check if file exists (i.e. non-unique subject+date+session).
    if exist([sav '.mat'],'file')
        % Ask to overwrite previous file
        choice = questdlg('Data file with the same name already exists. Would you like to proceed and overwrite?', ...
            'Overwrite data session file','Yes','No','No');
        switch choice
            case 'Yes'
                 warning('Proceeding to overwrite file...')
            case 'No'
                error('Program terminated. Please choose a new subject + session number combination.')
        end
    end

    %% Settings for log file
    writeLogFile = 1; % 1 to activate file logging
    if writeLogFile
        warning('Writing to log file: ON')
    else
        warning('Writing to log file: OFF (information will not be logged to log file)')
    end

    %% Set experimental parameters
    % Read from parameters text file
    parafile = fopen('parameters_v8_13.txt','r');

    A = textscan(parafile,'%s','Delimiter','');
    [str]=sscanf(A{1}{1},'%s%f');
    screenNumber=str(end);
    [str]=sscanf(A{1}{2},'%s%f');
    triangleSize=str(end);
    [str]=sscanf(A{1}{3},'%s%f');
    linearV=str(end);
    [str]=sscanf(A{1}{4},'%s%f');
    rotationV=str(end);
    [str]=sscanf(A{1}{5},'%s%f');
    targetRadius=str(end);
    [str]=sscanf(A{1}{6},'%s%f');
    targetSize=str(end);
    [str]=sscanf(A{1}{7},'%s%f');
    %minDistanceToTarget=str(end);%doesn't read from parameters file
    %anymore
    minDistanceToTarget=targetSize;
    [str]=sscanf(A{1}{8},'%s%f');
    stopTime=str(end);
    [str]=sscanf(A{1}{9},'%s%f');
    maxTrialTime=str(end);
    [str]=sscanf(A{1}{10},'%s%f');
    randExtMaxTime=str(end);
    [str]=sscanf(A{1}{11},'%s%f');
    interTrialTime=str(end);
    [str]=sscanf(A{1}{12},'%s%f');
    juiceConnected=str(end);
    [str]=sscanf(A{1}{13},'%s%f');
    temp=str(end);
    juicerPort=['COM' num2str(temp)];
    [str]=sscanf(A{1}{14},'%s%f');
    rewardTime=str(end);
    [str]=sscanf(A{1}{15},'%s%f');
    useArduinoJS=str(end);
    [str]=sscanf(A{1}{16},'%s%f');
    temp=str(end);
    arduinoPort=['COM' num2str(temp)];
    [str]=sscanf(A{1}{17},'%s%f');
    trialMode=str(end);
    [str]=sscanf(A{1}{18},'%s%f');
    early_termination_on=str(end);
    [str]=sscanf(A{1}{19},'%s%f');
    forward_guide_on=str(end);
    [str]=sscanf(A{1}{20},'%s%f');
    turning_guide_on=str(end);
    [str]=sscanf(A{1}{21},'%s%f');
    repeatUntilSuccess=str(end);
    [str]=sscanf(A{1}{22},'%s%f');
    use_guide_rails=str(end);
    [str]=sscanf(A{1}{23},'%s%f');
    narrowGuide=str(end);
    [str]=sscanf(A{1}{24},'%s%f');
    global randomBlock
    randomBlock=str(end);

    fclose(parafile);

    % Print summary of the session initiated, before program starts up
    disp([participant ', Session Number ' num2str(sessNum) ', mode ' num2str(trialMode) ' for ' num2str(numTrials) ' trials'])

catch ME1
    warning('Something went wrong in setting up session.')
    try
        fclose(parafile);
    catch
        fclose all;
    end
    rethrow(ME1)
end

%% Check/Initialize sound and COM ports before starting up trial

try
    % Test Juicer port
    if juiceConnected==1
        juiceReward=serial(juicerPort);
        fopen(juiceReward);
        pause(rewardTime);
        fclose(juiceReward);
    end
catch ME1R
    warning('Check juicerPort (in parameters.txt). Something went wrong initiating the Juicer.')
    try
        fclose(juiceReward);
    catch
        fclose all;
    end
    rethrow(ME1R)
end

try
    % Test Arduino port
    if useArduinoJS
        global arduino
        arduino=serial(arduinoPort,'BaudRate',115200);

        fopen(arduino);
        pause(2) % Wait to initialize port before reading data
        fwrite(arduino,'v','char'); %for activate data

    end

catch ME1A
    warning('Check Arduino COM port (Line 175 in main). Something went wrong initiating the Arduino.')
    try
        if useArduinoJS
            fclose(arduino);
        end
    catch
        fclose all;
    end
    rethrow(ME1A)
end

try
    % Check for audio output information
    soundinfo=audiodevinfo;
    if isempty(soundinfo.output)
        error('Unable to find audio output')
    end

    % Test sound playback
    [startsound,Fs1]=audioread('Windows Ding.wav');
    startsound=startsound*40;
    [norewardsound,Fs2]=audioread('ShortBeep.wav');
    norewardsound=norewardsound*2;
%     [rewardsound,Fs3]=audioread('click3.wav');
%     rewardsound=rewardsound*10;
    temp=round(size(startsound,1)/3);
    rewardsound=[startsound(1:temp,:) ;startsound(1:temp,:) ; startsound];
    Fs3=Fs1*2;
    sound(rewardsound,Fs3);

catch ME1S
    error('Check computer audio settings (speaker icon at bottom right). Something went wrong playing sounds.')
    rethrow(ME1S)
end

%% Setup and run trials
try
    %% Initialize trial structure
    targetReached=1; %logic type, to check if the target is been reached.
    timeElapsed=0;
    targetDuration = 0.5;
    target_drawn = 0;
    target_flipped = 0;
    target_onset = -1;
    global trialNum
    trialNum=1;
    sucTrialNum=1;
    global JSvalue
    JSvalue=[0 0];

    % Flags to activate if one of the following directions should be 'blocked'
    global blockForward
    blockForward = 0;
    global blockLeft
    blockLeft = 0;
    global blockRight
    blockRight = 0;

    %% Initialize session structure for saving later
    global session_data
    session_data.SessNum=sessNum;
    session_data.NumTrials=numTrials;
    session_data.ExpTime=clock;
    session_data.MaxTime=maxTrialTime;
    session_data.RewardTime=rewardTime;
    session_data.InterTrialTime=interTrialTime;
    session_data.TargetRadius=targetRadius;
    session_data.TriangleSize=triangleSize;
    session_data.TargetSize=targetSize;
    session_data.TargetLocation=zeros(numTrials,2);
    session_data.LinearVelocity=linearV;
    session_data.RotationVelocity=rotationV;
    session_data.Mode=trialMode;
    session_data.MinDist=minDistanceToTarget;
    session_data.TrialJS=cell(numTrials,1);
    session_data.TrialTiming=cell(numTrials,1);
    session_data.TrialFullTiming=cell(numTrials,1);
    session_data.TrialMoveDir=cell(numTrials,1);
    session_data.AllJSSignals=[];
    session_data.AllTiming=[];
    session_data.TrialOutcomes=zeros(numTrials,4);
    session_data.TrialOutcomesTitle={'Start time','End time','Success','Time taken'};
    session_data.CodeVersion=codeVer;
    if useArduinoJS
        session_data.InputType='ArduinoJoystick';
    else
        session_data.InputType='Keyboard';
    end
    session_data.Trajectory=[];
    session_data.Coordinate=[];
    session_data.TrajTiming=[];

    %% Set screen parameters
    BCI_sign_fz=10; % neuro signal frequency, the signal come in every 100ms
    time_interval=1/BCI_sign_fz; %0.1; % neuro sign frq is 10 Hz , therefor time _interveal is set at 0.1ms

    PsychDefaultSetup(2);% Here we call some default settings for setting up Psychtoolbox
    screens = Screen('Screens');% Get the screen numbers
    % Using screenNumber stated in parameters txt file
    %screenNumber = max(screens);% Draw to the external screen if avaliable

    white = WhiteIndex(screenNumber);% Define black and white
    black = BlackIndex(screenNumber);
    if ~debug_mode
        [window, windowRect] = PsychImaging('OpenWindow', screenNumber, black);% Open on screen window
    else
        [window, windowRect] = PsychImaging('OpenWindow', screenNumber, black, [0 0 960 720]);% Open on windowed mode (for debugging)
    end
    [screenXPixels, screenYPixels] = Screen('WindowSize', window);
    ifi = Screen('GetFlipInterval', window);% Query the frame duration
    Screen('Preference', 'SkipSyncTests', 1);
    Screen('BlendFunction', window, 'GL_SRC_ALPHA', 'GL_ONE_MINUS_SRC_ALPHA');% Set up alpha-blending for smooth (anti-aliased) lines
    screenXResolution=screenXPixels/35.57; % the wheelchair works in a 35.57 X 20 m room
    screenYResolution=screenYPixels/20;
    [xCenter, yCenter] = RectCenter(windowRect);% Get the centre coordinate of the window

    topPriorityLevel = MaxPriority(window); % Maximum priority level
    Priority(topPriorityLevel);

    lineWidthPix = 1.5;
    dotSizePix = 5; %originally 2
    dotColor = [0 1 1];

    esc = 1;% initialize 'esc'

    %% Note: this part of code assume left and right motors are same, using same PID control
    %setting wheelchair parameters
    confidence=1;%  correct BCI recogntion rate 0-1. 1 is no error
    l=0.8; % width of the wheelchair, this parameter shall match with the physical width of the wheelchair.
    R=0; % meter,turning radious, it is allowed to be zero. originally 1.0
    designMaxV=linearV; %m/s, set by user originally 0.5;
    V_left=designMaxV;
    V_right=designMaxV;
    omega= designMaxV/R; % angluar velocity. radians / second
    if omega>=1.0
        omega=rotationV; %rotation speed, set by user, originally set at 1
    end
    accel=3; % m/s^2, set the accel behavior of the wheelchair.

    % PID design parameter, they are the same for left and right motor
    kp_1=427.28; %% for fast response. the rise time =0.28s, settling time=0.54s, overshoot 0%
    ki_1=58.1;
    kd_1=0;
    % kp_1=53.78; %% for slow stopping rise time 2.25, settling time 4 overshoot 0%
    % ki_1=9.77;
    % kd_1=0;

    %% Form state space model for wheelchair with PID control
    % % if control license is not avaiable, please load ('wheel_model.mat'). if the
    % % parameter of the control model is modified, please do remember to update
    % % the state space model with the control tool box.
    % [A_1,B_1,C_1,D_1]=wheelchair_model(kp_1,ki_1,kd_1,designed_max_v,BCI_sign_fz);
    load('wheel_model.mat','A_1','B_1','C_1','D_1');

    % Create triangle (wheelchair)
    triangle=triangleSize*[-10 10; -10 -10; 20 0]; % Make a Triangle% the wheel chair in pixcel
    pentagonArrow=triangleSize*[10 10; -10 10; -10 -10; 10 -10; 20 0]; % Make a Triangle% the wheel chair in pixcel
    if useArrow
        centroid2Tip = pentagonArrow(5,1);
    else
        centroid2Tip = triangle(3,1);
    end
    trianColor = [1 1 0]; %color %  Set the color of the rect to red

    %% Start Graphics Timer
    global grabData; grabData=0;
    global keeptiming; keeptiming=tic;
    % Denote start of session
    if useArduinoJS
        fwrite(arduino,'c','char'); %Event 1
        fwrite(arduino,'b','char'); %Event 2
        fwrite(arduino,'a','char'); %Event 3
    end

    global trialtiming;
    TMRPERIOD_GRAPHICS = 0.1; %in seconds. fixed at 0.05s.
    TimerBusyMode_Queue = 0;% Timer busy mode: 1 for 'queue';  0 for 'drop'

    global updateFailed
    graphicsTimer = timer('TimerFcn', @(~,~)graphics_update_v8(), 'Period', TMRPERIOD_GRAPHICS);
    set(graphicsTimer, 'ExecutionMode', 'fixedRate');
    if TimerBusyMode_Queue
        set(graphicsTimer, 'BusyMode', 'queue');
    else
        set(graphicsTimer, 'BusyMode', 'drop');
    end
    start(graphicsTimer);

    %% Start the session
    i=0;
    BCI_signal=[];
    loopTime=0.1 ; % simulation loop time 0.1s
    startNewTrial=1;

    % Setup for trial modes 32 and 33 -- randomize across modes 1, 21 and 22
    randMode = 0; % for trial mode 32 and 33: randomize among 1, 21 and 22
    modeUsed = []; % for trial mode 32 and 33: keeps track of modes used
    recordTrialMode = trialMode;

    isSuccess = 1; % to track if previous trial succeeds

    global randSeq
    randSeq = [];

    if trialMode==32
        randMode = 1;
        allModes = [1, 21, 22];
    end

    earlyTermination = 0;
    isTerminated = 0;
    if early_termination_on
        earlyTermination = 1;
    end

%     while esc && ((trialNum<=numTrials && ~countNumSuccess) || (sucTrialNum<=numTrials && countNumSuccess)) % stop if 'esc' key pressed, or or reach pre-set trial number
    while esc && sucTrialNum<=numSucTrials % stop if 'esc' key pressed, or or reach pre-set trial number
        % Checking mechanism for function in timer (background)
        if updateFailed
            error('Graphic Update Failed')
        end

        if grabData
            if startNewTrial==1
                %Start new trial

                % Set target location
                if repeatUntilSuccess==0 || (repeatUntilSuccess && isSuccess)

                    if randomBlock && isempty(randSeq)
                        randSeq = randperm(12);
                        if trialMode == 3
                            randSeq = rem(randSeq,2);
                        else
                            randSeq = rem(randSeq,3);
                        end
                    end

                    % For trial modes 32 and 33: randomize selection of 1, 21 or 22
                    if randMode
                        if randomBlock
                            randIdx = randSeq(1)+1;
                            randSeq(1) = [];
                        else
                            randIdx = randperm(numel(allModes));
                        end
                        trialMode = allModes(randIdx(1));
                        modeUsed = [modeUsed trialMode];
                    end

                    [circleX, circleY] = setTargetLocation(trialMode,targetRadius,targetSize,triangleSize,yCenter,screenXResolution,screenYResolution,screenYPixels);
                    extTime = round(randExtMaxTime*rand());
                    isSuccess = 0;
                end

                % Reset/Initialize variable 'targetColor' here
                targetColor=[1 0 0]; %initialize to red
                session_data.TargetLocation(trialNum,:)=[circleX+xCenter,circleY];

                % Reset parameters
                targetReached=0;
                startNewTrial=0;
                desired_v_left=zeros(2,1);desired_v_right=zeros(2,1);
                signalLines=zeros(4, 100);% store BCI signal for display
                x_left = [0;0];x_right = [0;0];% x are the states in state space equation
                v_left = [0;0];v_right = [0;0];% v are the states in the state space eqaution
                acc_left=0;acc_right=0;
                omega_pid=[0;0];% omega is is angular velocity of the wheelchair
                x=0;y=0; % position of the wheelchair in 'meter'
                p_x=[];p_y=[];
                trX=[];trY=[];% position of the wheelchair in 'pixel' for display
                centroidX=[];centroidY=[];centroidTheta=[];% internal coordinate/bearing for wheelchair
                trTime=[]; % keeping track of time when tr and centroid are updated
                theta =[90*pi/180;0];% orientation of the wheelchair to x axis, intial at 90degree, pointing y+ axis
                counter1=0;% counter to keep track of inter trial time
                Screen('TextSize', window, 24);

                % Intertrial pause
                while counter1 <=interTrialTime

                    % TODO: This is the code for inter-trial pause screen
                    % It can be replaced with circle/cross (2 rectangles)
                    % that changes color based on the value of
                    % '(interTrialTime-counter1)'

                    str=['Trial No. ',num2str(sucTrialNum) ' starting in ' num2str(round(interTrialTime-counter1)),' seconds...'];
                    DrawFormattedText(window, str , screenXPixels/2-300,screenYPixels/2-400, [1 0 0]);

                    %fixation cross
                    fixCrossDimPix = 100;
                    xCoords = [-fixCrossDimPix fixCrossDimPix 0 0];
                    yCoords = [0 0 fixCrossDimPix -fixCrossDimPix];
                    allCoords = [xCoords; yCoords];
                    lineWidthPix = 10;%line width, max is 10.
                    if round(interTrialTime-counter1)==2
                        crosscolor=[1 0 0];
                    elseif round(interTrialTime-counter1)==1
                        crosscolor=[1 1 0];
                    elseif round(interTrialTime-counter1)==0
                        crosscolor=[0 1 0];
                    else
                        crosscolor=[0 0 1];
                    end
                    Screen('DrawLines', window, allCoords,lineWidthPix, crosscolor, [xCenter yCenter], 2);
                    Screen('Flip', window);% Flip to the screen


                    if useArduinoJS
%                         fwrite(arduino,'r','char');
%                         temp=fread(arduino,2,'int8');
%                         JSvalue=[temp(1) temp(2)];
                        joystick_detect_v8();
                    else
                        keyboard_detect_v8();
                    end

                    delaytime=0.25;
                    pause(delaytime);

                    % joystick must be in neutral position during intertrial pause
                    if JSvalue(1)<20 && JSvalue(1)>-20 && JSvalue(2)<20 && JSvalue(2)>-20
                        counter1=counter1+delaytime;
                    else
                        counter1=0;
                    end
                end

                %start of trial!
                sound(startsound,Fs1);
                disp(str)
                trialtiming=tic;
                rewardcounter=1; %delay for reward
                if useArduinoJS
                    fwrite(arduino,'b','char'); %Event 2
                    fwrite(arduino,'a','char'); %Event 3
                    fwrite(arduino,'c','char'); %Event 1
                end
                session_data.TrialOutcomes(trialNum,1)=toc(keeptiming);

                %condition for trialtermination
                JSstop=1; %start of trial, initialize to 1
            end %end of start new trial loop

            timeElapsed=toc(trialtiming);

            % Display Target
%             target_rad=targetSize; %size of target, originally 10
%             Screen('DrawDots', window,  [circleX; circleY], target_rad, targetColor,[xCenter 0], 2);
            SquareTarget=targetSize*[1 1; -1 1; -1 -1; 1 -1];
            targetLoc=[circleX+xCenter,circleY];
            if circleX>0
                targetLoc=[circleX+xCenter+20,circleY];
            elseif circleX<0
                targetLoc=[circleX+xCenter-20,circleY];
            end
            targetpoly=SquareTarget+[1;1;1;1]*targetLoc;
            if (target_drawn == 0) && (target_onset < 0.0)
              Screen('FillPoly', window, targetColor, targetpoly); %square target
              target_drawn = 1;
              target_flipped = 1;
            elseif timeElapsed - target_onset < targetDuration
              Screen('FillPoly', window, targetColor, targetpoly);
              target_flipped = 0;
              target_drawn = 1;
            elseif target_drawn ==1 && timeElapsed - target_onset >= targetDuration
                target_flipped = -1;
                target_drawn = 0;
            else
              target_flipped = 0;
            end

            % Update coordinates for wheelchair
            BCI_signal=[BCI_signal signalLines(:,1)];
            v_left(2,1)=v_left(1,1); % (2,1) contains velocity at t-1; (1,1) contains velocity at t
            v_right(2,1)=v_right(1,1);
            [x_left, v_left(1,1)]=calcu_vel(x_left,A_1,B_1,C_1,D_1,desired_v_left(1,1));
            [x_right, v_right(1,1)]=calcu_vel(x_right,A_1,B_1,C_1,D_1,desired_v_right(1,1));
            omega_pid(2,1)=omega_pid(1,1); % keep angular velocity at t-1
            omega_pid(1,1)= (v_right(1,1)-v_left(1,1))/l; % update current angluar velocity
            theta(2,1)=theta(1,1);% keep the angle data from t-1
            theta(1,1)=theta(2,1)+omega_pid(1,1)*loopTime+(-diff(omega_pid))*(loopTime^2)/2 ;% update the orentation of the wheelchir, express by theta
            v=(v_left+v_right)/2;
            dis=v(1,1)*loopTime+(-diff(v))*(loopTime^2)/2;
            x=x+dis*cos(theta(2,1));
            y=y+dis*sin(theta(2,1));

            X=x*screenXResolution;
            Y=y*screenYResolution;
            p_x=[p_x x];p_y=[p_y y];% store postion info;

            % Draw the wheelchair (pentagon arrow or triangle)
            if useArrow
                % -- pentagon --
                pentagon_p=pentagonArrow + [X Y;X Y;X Y;X Y;X Y]+ [xCenter yCenter;xCenter yCenter;xCenter yCenter;xCenter yCenter;xCenter yCenter];
                PL=pentagonArrow;%triangle-repmat(mean(triangle),[3 1]);% move triangle to centroid
                gamma=theta(1,1)*180/pi;
                rota_mat = [cosd(gamma) -sind(gamma) 0; sind(gamma) cosd(gamma) 0; 0 0 1];
                meanPL=[pentagon_p(5,1)-centroid2Tip,pentagon_p(5,2)];

                PL=[PL'; 0 0 0 0 0];   PL_=rota_mat*PL; PL_=PL_(1:2,:)';% rotate the triangle
                pentagon_rota=PL_+repmat(meanPL,[5 1]); % move back to the place

                % Draw the triangle to the screen
                for_display(:,1)=pentagon_rota(:,1);
                for_display(:,2)=repmat(screenYPixels,[5 1])-pentagon_rota(:,2);
                %position of tip of wheelchair
                xtip=for_display(5,1);
                ytip=for_display(5,2);
                % -- pentagon --
            else
                % -- triangle --
                triangle_p=triangle + [X Y;X Y;X Y]+ [xCenter yCenter;xCenter yCenter;xCenter yCenter];
                meanPL=mean(triangle_p);% find the centroid of the triangle
                PL=triangle;%triangle-repmat(mean(triangle),[3 1]);% move triangle to centroid
                gamma=theta(1,1)*180/pi;
                rota_mat = [cosd(gamma) -sind(gamma) 0; sind(gamma) cosd(gamma) 0; 0 0 1];

                PL=[PL'; 0 0 0];   PL_=rota_mat*PL; PL_=PL_(1:2,:)';% rotate the triangle
                triangle_rota=PL_+repmat(meanPL,[3 1]); % move back to the place

                % Draw the triangle to the screen
                for_display(:,1)=triangle_rota(:,1);
                for_display(:,2)=repmat(screenYPixels,[3 1])-triangle_rota(:,2);
                %position of tip of wheelchair
                xtip=for_display(3,1);
                ytip=for_display(3,2);
                % -- triangle --
            end

            Screen('FillPoly', window,trianColor, for_display);

            % Record points for trajectory.
            % (First 2 lines for center,
            % Subsequent 2 lines for tip)
%             trX=[trX mean(triangle_rota(:,1))];
%             trY=[trY screenYPixels-mean(triangle_rota(:,2))];% the reason for 'screenYpixels-mean' is to display on screen
            trX = [trX xtip];
            trY = [trY ytip];

            centroidX = [centroidX meanPL(1)];
            centroidY = [centroidY screenYPixels - meanPL(2)];
            centroidTheta = [centroidTheta gamma];

            trTime = [trTime toc(keeptiming)];

            % Update JSValue (in functions) using either joystick or keyboard
            if useArduinoJS
                joystick_detect_v8();
            else
                keyboard_detect_v8();
            end


            width=triangleSize;
            if narrowGuide
                width=triangleSize*25/screenXResolution;
            end

            % Display Guide Lines
            % If forward guide is on, and target is in front
            if forward_guide_on && (circleX == 0 && circleY < yCenter)
                Screen('DrawLine',window,[1 1 1],xCenter-width*screenXResolution,yCenter,xCenter-width*screenXResolution,yCenter-screenYPixels/2,3);
                Screen('DrawLine',window,[1 1 1],xCenter+width*screenXResolution,yCenter,xCenter+width*screenXResolution,yCenter-screenYPixels/2,3)
            end
            % If turning guide is on, and target is on either side
            if turning_guide_on && (circleX ~= 0 && circleY == yCenter)
                Screen('DrawLine',window,[1 1 1],xCenter-screenXPixels/2,yCenter-width*screenYResolution,xCenter+screenXPixels/2,yCenter-width*screenYResolution,3);
                Screen('DrawLine',window,[1 1 1],xCenter-screenXPixels/2,yCenter+width*screenYResolution,xCenter+screenXPixels/2,yCenter+width*screenYResolution,3);
            end

            % Draw the trajecotry of the wheelchair
            Screen('DrawDots', window, [trX; trY], dotSizePix, dotColor,[], 2);
            session_data.Trajectory{trialNum}=[trX; trY];
            session_data.Coordinate{trialNum}=[centroidX; centroidY; centroidTheta];
            session_data.TrajTiming{trialNum}=trTime;

            if use_guide_rails
                outOfBounds = check_for_limits(circleX, circleY, xtip, ytip, gamma, xCenter, yCenter, width*screenXResolution, width*screenYResolution);
            end

            % Convert input (JSValue) to command/info to update triangle
            [desired_v_left,desired_v_right,signalLines]= input_to_command(desired_v_left,desired_v_right,signalLines,V_left,V_right,accel,loopTime,l,omega,R,xtip,ytip,screenXPixels,screenYPixels,gamma);

            % Early Termination
            if earlyTermination
                % Check if trial should be terminated early
                isTerminated = check_for_limits(circleX, circleY, xtip, ytip, gamma, xCenter, yCenter, width*screenXResolution, width*screenYResolution);
                %print([gamma isTerminated]);
            end



            % Check if target is reached
            distanceToTarget=sqrt(((circleX+xCenter)-xtip)^2+((circleY)-ytip)^2); % between tip of triangle
            %dist_to_targ=sqrt(((circle_x+xCenter)-trX(1,end))^2+((circle_y)-trY(1,end))^2); % between center of triangle
            %disp(dist_to_targ); %for debug
            targetReached=distanceToTarget<minDistanceToTarget; %logic

            % This is where the countdown starts. We can change the
            % variable 'targetColor' here.
            if targetReached==1
                rewardcounter=rewardcounter+1;
                targetColor=[0 1 0];
                %draw
                Screen('FillPoly', window, targetColor, targetpoly);

            else
                rewardcounter=1;
                targetColor=[1 0 0];
                %dont draw
            end

            %Flip everything to screen
            Screen('Flip', window);% Flip to the screen
            if target_flipped
              target_onset = toc(trialtiming);
            end
            % Check for JSstop
            temp=JSvalue(1)<20 && JSvalue(1)>-20 && JSvalue(2)<20 && JSvalue(2)>-20;
            if targetReached==1 && temp
                JSprev=JSstop;
                JSstop=2;
            elseif targetReached==1 && ~temp
                JSprev=JSstop;
                JSstop=3;
            else
                JSprev=JSstop;
            end
            if JSprev==2 && JSstop==3
                isTerminated=1;
            end


            % Check if escape key or 'k' key is pressed
            [keyIsDownK, ~, keyCodeK] = KbCheck;
            if keyIsDownK && keyCodeK(escapeKey)
                esc=0;
            end

%             if targetReached || timeElapsed>=maxTrialTime || keyCodeK(kKey); % check the distance between the target and wheelchair, 20 pix length || time run out for the trial

            %%display to screen, move on to next trialkkk
            if timeElapsed>=maxTrialTime+extTime; % run off time (extTime is random length of 'extension time')

                %red cross picture
                theImage = imread('cross.jpg');
                [s1, s2, s3] = size(theImage);
                imageTexture = Screen('MakeTexture', window, theImage);
                Screen('DrawTexture', window, imageTexture, [], [], 0);

                Screen('TextSize', window, 24);
                str=['Time runs out for Trial No.: ' num2str(trialNum) '.'];
                DrawFormattedText(window, str, screenXPixels/2-300, screenYPixels/2-400, [1 0 0]);
                Screen('Flip', window);% Flip to the screen
                sound(norewardsound,Fs2);
                session_data.TrialOutcomes(trialNum,2)=toc(keeptiming);
                session_data.TrialOutcomes(trialNum,4)=toc(trialtiming);
                if useArduinoJS
                    fwrite(arduino,'a','char'); %Event 3
                end
                startNewTrial=1;
            elseif isTerminated

                % This is the code for early terminated trial.
                Screen('TextSize', window, 24);
                str=['Trial No.: ' num2str(trialNum) ' was terminated early.'];
                DrawFormattedText(window, str, screenXPixels/2-300, screenYPixels/2-400, [1 0 0]);

                %red cross picture
                theImage = imread('cross.jpg');
                [s1, s2, s3] = size(theImage);
                imageTexture = Screen('MakeTexture', window, theImage);
                Screen('DrawTexture', window, imageTexture, [], [], 0);

                Screen('Flip', window);% Flip to the screen
                sound(norewardsound,Fs2);
                session_data.TrialOutcomes(trialNum,2)=toc(keeptiming);
                session_data.TrialOutcomes(trialNum,4)=toc(trialtiming);
                if useArduinoJS
                    fwrite(arduino,'a','char'); %Event 3
                end
                startNewTrial=1;
            elseif keyCodeK(kKey) % press 'k' to kill the trial

                %red cross picture
                theImage = imread('cross.jpg');
                [s1, s2, s3] = size(theImage);
                imageTexture = Screen('MakeTexture', window, theImage);
                Screen('DrawTexture', window, imageTexture, [], [], 0);

                Screen('TextSize', window, 24);
                str=['Trial No.: ' num2str(trialNum), ' was terminated by user.'];
                DrawFormattedText(window, str , screenXPixels/2-300, screenYPixels/2-400, [1 0 0]);
                Screen('Flip', window);% Flip to the screen, why need to filp screen
                sound(norewardsound,Fs2);
                session_data.TrialOutcomes(trialNum,2)=toc(keeptiming);
                session_data.TrialOutcomes(trialNum,4)=toc(trialtiming);
                if useArduinoJS
                    fwrite(arduino,'a','char'); %Event 3
                end
                startNewTrial=1;

            elseif rewardcounter>=stopTime/TMRPERIOD_GRAPHICS
                % green smiley face
                theImage = imread('smiley.jpg');
                [s1, s2, s3] = size(theImage);
                imageTexture = Screen('MakeTexture', window, theImage);
                Screen('DrawTexture', window, imageTexture, [], [], 0);


                Screen('TextSize', window, 36);
                str=['Trial No.:' num2str(trialNum) ' success!'];
                DrawFormattedText(window, str, screenXPixels/2-300, screenYPixels/2-400, [0 1 0]);
                Screen('Flip', window);% Flip to the screen, why need to filp screen
                session_data.TrialOutcomes(trialNum,2)=toc(keeptiming);
                session_data.TrialOutcomes(trialNum,3)=1;
                session_data.TrialOutcomes(trialNum,4)=toc(trialtiming);
%                 if useArduinoJS
%                     fwrite(arduino,'a','char'); %Event 3
%                 end
                isSuccess = 1;

                % Give Reward
                if useArduinoJS
                    fwrite(arduino,'b','char'); % Event 2
                end
                sound(rewardsound,Fs3);
                if juiceConnected==1
                    fopen(juiceReward);
                    pause(rewardTime);
                    fclose(juiceReward);
                end
                startNewTrial=1;
            end

            if startNewTrial
                save(sav,'session_data');
                target_onset = -1;
                target_drawn = 0;
                target_flipped = 0;
                pause (interTrialTime);
                trialNum=trialNum+1;
                if isSuccess
                    sucTrialNum=sucTrialNum+1;
                end
            end
%             end

        end
    end

    % For trial modes 32 and 33: set trial mode back to original value (for saving to file)
    trialMode = recordTrialMode;

    %% End of session
    if isvalid(graphicsTimer)
        stop(graphicsTimer);
        disp('### Stop graphicsTimer! ###\n');
        pause(0.5);
        delete(graphicsTimer);
        Screen('CloseAll');
    end

    % Denote end of session
    if useArduinoJS
        fwrite(arduino,'a','char'); %Event 3
        fwrite(arduino,'b','char'); %Event 2
    end

    if useArduinoJS
        disp('Closing Arduino...');
        fclose(arduino);
    end

    Screen('CloseAll');
    disp('Session complete!');


catch ME2
    warning('Something went wrong during the training session. Closing Arduino & RewardPort...')
    % For trial modes 32 and 33: set trial mode back to original value (for saving to file)
    if exist('randMode','var')
        if randMode
            modeUsed(end) = [];
            trialMode = recordTrialMode;
        end
    end
    try
        if useArduinoJS
            fclose(arduino);
            warning('Arduino Closed!');
        end
        if juiceConnected
            fclose(juiceReward);
            warning('RewardPort Closed!');
        end
    catch
        fclose all;
    end

    % From ErrorCatchDemo in Psychtoolbox Demos
    % If an error occurs, the catch statements executed.  We restore as
    % best we can and then rethrow the error so user can see what it was.
    sca;
    oldVisualDebugLevel = Screen('Preference', 'VisualDebugLevel', 3);
    oldSupressAllWarnings = Screen('Preference', 'SuppressAllWarnings', 1);
    Screen('Preference', 'VisualDebugLevel', oldVisualDebugLevel);
    Screen('Preference', 'SuppressAllWarnings', oldSupressAllWarnings);
    if exist('graphicsTimer')
        if isvalid(graphicsTimer)
            stop(graphicsTimer);
            disp('### Stop graphicsTimer! ###\n');
            pause(0.5);
            delete(graphicsTimer);
            Screen('CloseAll');
        end
    end
    fprintf('We''ve hit an error.\n');
end

%% Create log file at the end of trial
try
    if writeLogFile
        % Create/Append to log file
        logFilename = ['log/note', dateString, '.txt'];
        logFile = fopen(logFilename,'a+');

        % Save details
        fprintf(logFile, '%s', participant);
        fprintf(logFile, ' ');
        fprintf(logFile, '%s', dateString);
        fprintf(logFile, '\r\n');

        currentTime=clock;
        timeEndString=[num2str(currentTime(4),'%02d') ':' num2str(currentTime(5),'%02d') ':' num2str(floor(currentTime(6)),'%02d')];
        fprintf(logFile, 'Time Start: %s; Time End: %s', timeStartString, timeEndString);
        fprintf(logFile, '\r\n');

        fprintf(logFile, 'Session ');
        fprintf(logFile, '%s', num2str(sessNum));
        fprintf(logFile, ': Trial Type ');

        %%
        switch trialMode
            case 1
                modeTypeString = 'only forward';
            case 2
                modeTypeString = 'turn left/right';
            case 21
                modeTypeString = 'turn right';
            case 22
                modeTypeString = 'turn left';
            case 3
                modeTypeString = 'turn left/right + forward (target on left/right)';
            case 31
                modeTypeString = 'either turn left/turn right or move straight';
            case 32
                modeTypeString = 'randomize modes 1, 21 and 22, and repeat until success';
                modeRecordString = ['Modes used:' mat2str(modeUsed)];
            case 33
                modeTypeString = 'randomize modes 1, 21 and 22 (regardless outcome)';
                modeRecordString = ['Modes used:' mat2str(modeUsed)];
            case 4
                modeTypeString = 'turn left/right + forward (target at any location)';
            otherwise
                modeTypeString = 'error finding mode';
        end
        fprintf(logFile, '%s', modeTypeString);
        fprintf(logFile, '\r\n');
        if exist('modeRecordString','var')
            fprintf(logFile, '%s', modeRecordString);
            fprintf(logFile, '\r\n');
        end

        if exist('ME2','var')
            fprintf(logFile, '!!! Session terminated due to an error.\r\n');
        end
        if esc==0
            fprintf(logFile, '!!! Session terminated with escape key.\r\n');
        end

        % Open an input dialog to ask for session comments
        comments = inputdlg('Session Comments:', 'User Comments', [1 100]);

        fprintf(logFile, 'Comments: ');
        fprintf(logFile, '%s', comments{:});
        fprintf(logFile, '\r\n\r\n\r\n');
        fclose(logFile);

        disp('File logged!');

        if exist('ME2','var')
            rethrow(ME2)
        end
    end
catch ME3
    if exist('ME2','var')
        psychrethrow(psychlasterror);
    end
    warning('Unable to write log file');
    try
        fclose(logFile);
    catch
        fclose all;
    end
    rethrow(ME3)
end

% process_session_data
