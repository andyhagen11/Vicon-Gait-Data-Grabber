%% Vicon Nexus Data Grabber %%
% Originating Author: Andrew Hagen
% Last Revised: 10/13/2023

% This script uses data from Vicon Nexus during a split-belt treadmill adaptation paradigm to create a structure of kinematic and kinetic data organized with each row in the final matrix being representative of one gait cycle

% Creates multi-layer nested data structure that is additive for all participants with a total of 336 matrices for each participant
    % Additionally pumps out another data structure with each gait cycle interpolated to be 100 data points long

    %Layer 1 = Participant
    %Layer 2 = Phase/Condition (Baseline, Adaptation1 (10min), Catch, Adaptation2 (5min) - will have to run separately for each phase since they are separate Vicon trials
        %Can remove this layer if not using different phases (conditions) - Just need to delete TrialName variable from structure in the export section
    %Layer 3 = More affected or Less affected
        %If not working with a paretic population just choose the same 'affected side' for everyone as this affects how asymmetries as calculated (MA - LA).
    %Layer 4 = Joint/segment/force (7 options)
    %Layer 5 = Stance or Swing
    %Layer 6 = X, Y or Z plane
    %Layer 7 = Data with each row being one gait cycle

%% Script initialization

% Must have a Vicon Nexus trail open on this machine
    % This trail must have gait events (heel strike and toe off) labeled and the dynamic gait model processed
    % Using the MATLAB integration in the Nexus pipeline makes this easy
   
% Variable abreviations
    % MA = More Affected Limb
    % LA = Less Affected limb

clc 
clear 
close all

%% Set up connection with Vicon Nexus and set path

    %Make sure Nexus is open with the desired trial 
    vicon = ViconNexus();
    
    SubjectName = vicon.GetSubjectNames;
    SubNum = char(SubjectName);
    Rate = vicon.GetFrameRate;
    Time = 1/Rate;
    Trial = inputdlg('Trial Name: ');
    Alimb = inputdlg('More Affected Limb? (L or R): '); %If not working with a paretic population just choose the same 'affected side' for everyone as this effects how asymmetries as calculated (MA - LA).
    TrialName = char(Trial);

     %PATH = pwd;
     %RootPath = uigetdir(PATH); % Self select path if you would like
     RootPath = 'R:\SBMS Joint Coordination\Raw Data\';
     cd(RootPath)

%% Import Heel Strike and Toe Off times - these come in frames (named according to affected limb)

if strcmp(Alimb,'L')
    MA_HeelStrikes = vicon.GetEvents(SubNum, 'Left', 'Foot Strike');
    LA_HeelStrikes = vicon.GetEvents(SubNum, 'Right', 'Foot Strike');
    MA_ToeOffs = vicon.GetEvents(SubNum, 'Left', 'Foot Off');
    LA_ToeOffs = vicon.GetEvents(SubNum, 'Right', 'Foot Off');

elseif strcmp(Alimb,'R')
    LA_HeelStrikes = vicon.GetEvents(SubNum, 'Left', 'Foot Strike');
    MA_HeelStrikes = vicon.GetEvents(SubNum, 'Right', 'Foot Strike');
    LA_ToeOffs = vicon.GetEvents(SubNum, 'Left', 'Foot Off');
    MA_ToeOffs = vicon.GetEvents(SubNum, 'Right', 'Foot Off');

end

%% Import data (named according to affected limb)

if strcmp(Alimb,'L')
    MA_AnkleAng = vicon.GetModelOutput(SubNum, 'LAnkleAngles'); 
    LA_AnkleAng = vicon.GetModelOutput(SubNum, 'RAnkleAngles');
    MA_KneeAng = vicon.GetModelOutput(SubNum, 'LKneeAngles');
    LA_KneeAng = vicon.GetModelOutput(SubNum, 'RKneeAngles');
    MA_HipAng = vicon.GetModelOutput(SubNum, 'LHipAngles');
    LA_HipAng = vicon.GetModelOutput(SubNum, 'RHipAngles');
    MA_FemurSeg = vicon.GetModelOutput(SubNum, 'LFE'); 
    LA_FemurSeg = vicon.GetModelOutput(SubNum, 'RFE');
    MA_TibSeg = vicon.GetModelOutput(SubNum, 'LTI');
    LA_TibSeg = vicon.GetModelOutput(SubNum, 'RTI');
    MA_FootSeg = vicon.GetModelOutput(SubNum, 'LFO');
    LA_FootSeg = vicon.GetModelOutput(SubNum, 'RFO');
    LA_Fx = vicon.GetDeviceChannel(1,1,1);
    LA_Fy = vicon.GetDeviceChannel(1,1,2);
    LA_Fz = vicon.GetDeviceChannel(1,1,3);
    MA_Fx = vicon.GetDeviceChannel(2,1,1);
    MA_Fy = vicon.GetDeviceChannel(2,1,2);
    MA_Fz = vicon.GetDeviceChannel(2,1,3);

elseif strcmp(Alimb,'R')
    LA_AnkleAng = vicon.GetModelOutput(SubNum, 'LAnkleAngles'); 
    MA_AnkleAng = vicon.GetModelOutput(SubNum, 'RAnkleAngles');
    LA_KneeAng = vicon.GetModelOutput(SubNum, 'LKneeAngles');
    MA_KneeAng = vicon.GetModelOutput(SubNum, 'RKneeAngles');
    LA_HipAng = vicon.GetModelOutput(SubNum, 'LHipAngles');
    MA_HipAng = vicon.GetModelOutput(SubNum, 'RHipAngles');
    LA_FemurSeg = vicon.GetModelOutput(SubNum, 'LFE'); 
    MA_FemurSeg = vicon.GetModelOutput(SubNum, 'RFE');
    LA_TibSeg = vicon.GetModelOutput(SubNum, 'LTI');
    MA_TibSeg = vicon.GetModelOutput(SubNum, 'RTI');
    LA_FootSeg = vicon.GetModelOutput(SubNum, 'LFO');
    MA_FootSeg = vicon.GetModelOutput(SubNum, 'RFO');
    MA_Fx = vicon.GetDeviceChannel(1,1,1);
    MA_Fy = vicon.GetDeviceChannel(1,1,2);
    MA_Fz = vicon.GetDeviceChannel(1,1,3);
    LA_Fx = vicon.GetDeviceChannel(2,1,1);
    LA_Fy = vicon.GetDeviceChannel(2,1,2);
    LA_Fz = vicon.GetDeviceChannel(2,1,3);

end

    % Make force output in all planes into one matrix (like for joints)
    % Flipped X and Y to match coordinate system of joints (e.g. Row 1 is flexion/extension for joint and propulsion/braking for force)
    MA_Force = [MA_Fy; MA_Fx; MA_Fz];
    LA_Force = [LA_Fy; LA_Fx; LA_Fz];

  disp('Data imported successfully.');

%% Make a structure of each joint in all 3 planes for stance and swing for the more affected leg

disp('Creating data structure...');

% More Affected Ankle Angles
   MA_AnkleAng_Swing = struct('X', nan((length(MA_HeelStrikes)-1),300), 'Y', nan((length(MA_HeelStrikes)-1),300), 'Z', nan((length(MA_HeelStrikes)-1),300));
   for i = 1:(length(MA_HeelStrikes)-1)
       TempWindow = MA_AnkleAng(1:3,MA_ToeOffs(i):MA_HeelStrikes(i)); % Find window of angles from each swing phase (toe off to heel strike)
       MA_AnkleAng_Swing.X(i,1:length(TempWindow)) = (TempWindow(1,:));
       MA_AnkleAng_Swing.Y(i,1:length(TempWindow)) = (TempWindow(2,:));
       MA_AnkleAng_Swing.Z(i,1:length(TempWindow)) = (TempWindow(3,:));
   end

   MA_AnkleAng_Stance = struct('X', nan((length(MA_HeelStrikes)-1),300), 'Y', nan((length(MA_HeelStrikes)-1),300), 'Z', nan((length(MA_HeelStrikes)-1),300));
   for i = 1:(length(MA_HeelStrikes)-1)
       TempWindow = MA_AnkleAng(1:3,MA_HeelStrikes(i):MA_ToeOffs(i+1)); % Find window of angles from each stance phase (heel strike to toe off)
       MA_AnkleAng_Stance.X(i,1:length(TempWindow)) = (TempWindow(1,:));
       MA_AnkleAng_Stance.Y(i,1:length(TempWindow)) = (TempWindow(2,:));
       MA_AnkleAng_Stance.Z(i,1:length(TempWindow)) = (TempWindow(3,:));
   end
    
   MA_AnkleAngs = struct('Stance', MA_AnkleAng_Stance, 'Swing', MA_AnkleAng_Swing);

% More Affected Knee Angles
   MA_KneeAng_Swing = struct('X', nan((length(MA_HeelStrikes)-1),300), 'Y', nan((length(MA_HeelStrikes)-1),300), 'Z', nan((length(MA_HeelStrikes)-1),300));
   for i = 1:(length(MA_HeelStrikes)-1)
       TempWindow = MA_KneeAng(1:3,MA_ToeOffs(i):MA_HeelStrikes(i)); % Find window of angles from each swing phase (toe off to heel strike)
       MA_KneeAng_Swing.X(i,1:length(TempWindow)) = (TempWindow(1,:));
       MA_KneeAng_Swing.Y(i,1:length(TempWindow)) = (TempWindow(2,:));
       MA_KneeAng_Swing.Z(i,1:length(TempWindow)) = (TempWindow(3,:));
   end

   MA_KneeAng_Stance = struct('X', nan((length(MA_HeelStrikes)-1),300), 'Y', nan((length(MA_HeelStrikes)-1),300), 'Z', nan((length(MA_HeelStrikes)-1),300));
   for i = 1:(length(MA_HeelStrikes)-1)
       TempWindow = MA_KneeAng(1:3,MA_HeelStrikes(i):MA_ToeOffs(i+1)); % Find window of angles from each stance phase (heel strike to toe off)
       MA_KneeAng_Stance.X(i,1:length(TempWindow)) = (TempWindow(1,:));
       MA_KneeAng_Stance.Y(i,1:length(TempWindow)) = (TempWindow(2,:));
       MA_KneeAng_Stance.Z(i,1:length(TempWindow)) = (TempWindow(3,:));
   end
    
   MA_KneeAngs = struct('Stance', MA_KneeAng_Stance, 'Swing', MA_KneeAng_Swing);

 % More Affected Hip Angles
   MA_HipAng_Swing = struct('X', nan((length(MA_HeelStrikes)-1),300), 'Y', nan((length(MA_HeelStrikes)-1),300), 'Z', nan((length(MA_HeelStrikes)-1),300));
   for i = 1:(length(MA_HeelStrikes)-1)
       TempWindow = MA_HipAng(1:3,MA_ToeOffs(i):MA_HeelStrikes(i)); % Find window of angles from each swing phase (toe off to heel strike)
       MA_HipAng_Swing.X(i,1:length(TempWindow)) = (TempWindow(1,:));
       MA_HipAng_Swing.Y(i,1:length(TempWindow)) = (TempWindow(2,:));
       MA_HipAng_Swing.Z(i,1:length(TempWindow)) = (TempWindow(3,:));
   end

   MA_HipAng_Stance = struct('X', nan((length(MA_HeelStrikes)-1),300), 'Y', nan((length(MA_HeelStrikes)-1),300), 'Z', nan((length(MA_HeelStrikes)-1),300));
   for i = 1:(length(MA_HeelStrikes)-1)
       TempWindow = MA_HipAng(1:3,MA_HeelStrikes(i):MA_ToeOffs(i+1)); % Find window of angles from each stance phase (heel strike to toe off)
       MA_HipAng_Stance.X(i,1:length(TempWindow)) = (TempWindow(1,:));
       MA_HipAng_Stance.Y(i,1:length(TempWindow)) = (TempWindow(2,:));
       MA_HipAng_Stance.Z(i,1:length(TempWindow)) = (TempWindow(3,:));
   end
    
   MA_HipAngs = struct('Stance', MA_HipAng_Stance, 'Swing', MA_HipAng_Swing);

%% Make a structure of each segment in all 3 planes for stance and swing for the more affected leg

% More Affected Femur Segments
   MA_FemurSeg_Swing = struct('X', nan((length(MA_HeelStrikes)-1),300), 'Y', nan((length(MA_HeelStrikes)-1),300), 'Z', nan((length(MA_HeelStrikes)-1),300));
   for i = 1:(length(MA_HeelStrikes)-1)
       TempWindow = MA_FemurSeg(1:3,MA_ToeOffs(i):MA_HeelStrikes(i)); % Find window of angles from each swing phase (toe off to heel strike)
       MA_FemurSeg_Swing.X(i,1:length(TempWindow)) = (TempWindow(1,:));
       MA_FemurSeg_Swing.Y(i,1:length(TempWindow)) = (TempWindow(2,:));
       MA_FemurSeg_Swing.Z(i,1:length(TempWindow)) = (TempWindow(3,:));
   end

   MA_FemurSeg_Stance = struct('X', nan((length(MA_HeelStrikes)-1),300), 'Y', nan((length(MA_HeelStrikes)-1),300), 'Z', nan((length(MA_HeelStrikes)-1),300));
   for i = 1:(length(MA_HeelStrikes)-1)
       TempWindow = MA_FemurSeg(1:3,MA_HeelStrikes(i):MA_ToeOffs(i+1)); % Find window of angles from each stance phase (heel strike to toe off)
       MA_FemurSeg_Stance.X(i,1:length(TempWindow)) = (TempWindow(1,:));
       MA_FemurSeg_Stance.Y(i,1:length(TempWindow)) = (TempWindow(2,:));
       MA_FemurSeg_Stance.Z(i,1:length(TempWindow)) = (TempWindow(3,:));
   end
    
   MA_FemurSegs = struct('Stance', MA_FemurSeg_Stance, 'Swing', MA_FemurSeg_Swing);

% More Affected Tibia Segments
   MA_TibSeg_Swing = struct('X', nan((length(MA_HeelStrikes)-1),300), 'Y', nan((length(MA_HeelStrikes)-1),300), 'Z', nan((length(MA_HeelStrikes)-1),300));
   for i = 1:(length(MA_HeelStrikes)-1)
       TempWindow = MA_TibSeg(1:3,MA_ToeOffs(i):MA_HeelStrikes(i)); % Find window of angles from each swing phase (toe off to heel strike)
       MA_TibSeg_Swing.X(i,1:length(TempWindow)) = (TempWindow(1,:));
       MA_TibSeg_Swing.Y(i,1:length(TempWindow)) = (TempWindow(2,:));
       MA_TibSeg_Swing.Z(i,1:length(TempWindow)) = (TempWindow(3,:));
   end

   MA_TibSeg_Stance = struct('X', nan((length(MA_HeelStrikes)-1),300), 'Y', nan((length(MA_HeelStrikes)-1),300), 'Z', nan((length(MA_HeelStrikes)-1),300));
   for i = 1:(length(MA_HeelStrikes)-1)
       TempWindow = MA_TibSeg(1:3,MA_HeelStrikes(i):MA_ToeOffs(i+1)); % Find window of angles from each stance phase (heel strike to toe off)
       MA_TibSeg_Stance.X(i,1:length(TempWindow)) = (TempWindow(1,:));
       MA_TibSeg_Stance.Y(i,1:length(TempWindow)) = (TempWindow(2,:));
       MA_TibSeg_Stance.Z(i,1:length(TempWindow)) = (TempWindow(3,:));
   end
    
   MA_TibSegs = struct('Stance', MA_TibSeg_Stance, 'Swing', MA_TibSeg_Swing);

 % More Affected Foot Segments
   MA_FootSeg_Swing = struct('X', nan((length(MA_HeelStrikes)-1),300), 'Y', nan((length(MA_HeelStrikes)-1),300), 'Z', nan((length(MA_HeelStrikes)-1),300));
   for i = 1:(length(MA_HeelStrikes)-1)
       TempWindow = MA_FootSeg(1:3,MA_ToeOffs(i):MA_HeelStrikes(i)); % Find window of angles from each swing phase (toe off to heel strike)
       MA_FootSeg_Swing.X(i,1:length(TempWindow)) = (TempWindow(1,:));
       MA_FootSeg_Swing.Y(i,1:length(TempWindow)) = (TempWindow(2,:));
       MA_FootSeg_Swing.Z(i,1:length(TempWindow)) = (TempWindow(3,:));
   end

   MA_FootSeg_Stance = struct('X', nan((length(MA_HeelStrikes)-1),300), 'Y', nan((length(MA_HeelStrikes)-1),300), 'Z', nan((length(MA_HeelStrikes)-1),300));
   for i = 1:(length(MA_HeelStrikes)-1)
       TempWindow = MA_FootSeg(1:3,MA_HeelStrikes(i):MA_ToeOffs(i+1)); % Find window of angles from each stance phase (heel strike to toe off)
       MA_FootSeg_Stance.X(i,1:length(TempWindow)) = (TempWindow(1,:));
       MA_FootSeg_Stance.Y(i,1:length(TempWindow)) = (TempWindow(2,:));
       MA_FootSeg_Stance.Z(i,1:length(TempWindow)) = (TempWindow(3,:));
   end
    
   MA_FootSegs = struct('Stance', MA_FootSeg_Stance, 'Swing', MA_FootSeg_Swing);
   
%% Make a strucutre of stance force in each plane for the more affected leg
% Use 10 * i because force is collected at 1000 Hz rather than 100 Hz for angles and events - the correct force index will be the angle index * 10
% (e.g. there if there is a heel strike at frame 55 for angles (100 Hz) the same heelstrike will occur at frame 550 for forces (1000 Hz)

   MA_Force_Stance = struct('X', nan((length(MA_HeelStrikes)-1),3000), 'Y', nan((length(MA_HeelStrikes)-1),3000), 'Z', nan((length(MA_HeelStrikes)-1),3000));
   for i = 1:(length(MA_HeelStrikes)-1)
       TempWindow = MA_Force(1:3,(10*(MA_HeelStrikes(i))):(10*(MA_ToeOffs(i+1)))); % Find window of forces from each stance phase (heel strike to toe off)
       MA_Force_Stance.X(i,1:length(TempWindow)) = (TempWindow(1,:)); % Propoulsion/braking forces
       MA_Force_Stance.Y(i,1:length(TempWindow)) = (TempWindow(2,:)); % Medial/lateral forces
       MA_Force_Stance.Z(i,1:length(TempWindow)) = (TempWindow(3,:)); % Vertical forces
   end

   MA_Forces = struct('Stance', MA_Force_Stance);


%% Create more affected leg structure
    More_Affected = struct('Hip_Angles', MA_HipAngs, 'Knee_Angles', MA_KneeAngs, 'Ankle_Angles', MA_AnkleAngs, 'Femur_Segment', MA_FemurSegs, 'Tibia_Segment', MA_TibSegs, 'Foot_Segment', MA_FootSegs, 'Forces', MA_Forces);

%% Make a structure of each joint in all 3 planes for stance and swing for the less affected leg

% Less Affected Ankle Angles
   LA_AnkleAng_Swing = struct('X', nan((length(LA_HeelStrikes)-1),300), 'Y', nan((length(LA_HeelStrikes)-1),300), 'Z', nan((length(LA_HeelStrikes)-1),300));
   for i = 1:(length(LA_HeelStrikes)-1)
       TempWindow = LA_AnkleAng(1:3,LA_ToeOffs(i):LA_HeelStrikes(i)); % Find window of angles from each swing phase (toe off to heel strike)
       LA_AnkleAng_Swing.X(i,1:length(TempWindow)) = (TempWindow(1,:));
       LA_AnkleAng_Swing.Y(i,1:length(TempWindow)) = (TempWindow(2,:));
       LA_AnkleAng_Swing.Z(i,1:length(TempWindow)) = (TempWindow(3,:));
   end

   LA_AnkleAng_Stance = struct('X', nan((length(LA_HeelStrikes)-1),300), 'Y', nan((length(LA_HeelStrikes)-1),300), 'Z', nan((length(LA_HeelStrikes)-1),300));
   for i = 1:(length(LA_HeelStrikes)-1)
       TempWindow = LA_AnkleAng(1:3,LA_HeelStrikes(i):LA_ToeOffs(i+1)); % Find window of angles from each stance phase (heel strike to toe off)
       LA_AnkleAng_Stance.X(i,1:length(TempWindow)) = (TempWindow(1,:));
       LA_AnkleAng_Stance.Y(i,1:length(TempWindow)) = (TempWindow(2,:));
       LA_AnkleAng_Stance.Z(i,1:length(TempWindow)) = (TempWindow(3,:));
   end
    
   LA_AnkleAngs = struct('Stance', LA_AnkleAng_Stance, 'Swing', LA_AnkleAng_Swing);

% Less Affected Knee Angles
   LA_KneeAng_Swing = struct('X', nan((length(LA_HeelStrikes)-1),300), 'Y', nan((length(LA_HeelStrikes)-1),300), 'Z', nan((length(LA_HeelStrikes)-1),300));
   for i = 1:(length(LA_HeelStrikes)-1)
       TempWindow = LA_KneeAng(1:3,LA_ToeOffs(i):LA_HeelStrikes(i)); % Find window of angles from each swing phase (toe off to heel strike)
       LA_KneeAng_Swing.X(i,1:length(TempWindow)) = (TempWindow(1,:));
       LA_KneeAng_Swing.Y(i,1:length(TempWindow)) = (TempWindow(2,:));
       LA_KneeAng_Swing.Z(i,1:length(TempWindow)) = (TempWindow(3,:));
   end

   LA_KneeAng_Stance = struct('X', nan((length(LA_HeelStrikes)-1),300), 'Y', nan((length(LA_HeelStrikes)-1),300), 'Z', nan((length(LA_HeelStrikes)-1),300));
   for i = 1:(length(LA_HeelStrikes)-1)
       TempWindow = LA_KneeAng(1:3,LA_HeelStrikes(i):LA_ToeOffs(i+1)); % Find window of angles from each sstance phase (heel strike to toe off)
       LA_KneeAng_Stance.X(i,1:length(TempWindow)) = (TempWindow(1,:));
       LA_KneeAng_Stance.Y(i,1:length(TempWindow)) = (TempWindow(2,:));
       LA_KneeAng_Stance.Z(i,1:length(TempWindow)) = (TempWindow(3,:));
   end
    
   LA_KneeAngs = struct('Stance', LA_KneeAng_Stance, 'Swing', LA_KneeAng_Swing);

 % Less Affected Hip Angles
   LA_HipAng_Swing = struct('X', nan((length(LA_HeelStrikes)-1),300), 'Y', nan((length(LA_HeelStrikes)-1),300), 'Z', nan((length(LA_HeelStrikes)-1),300));
   for i = 1:(length(LA_HeelStrikes)-1)
       TempWindow = LA_HipAng(1:3,LA_ToeOffs(i):LA_HeelStrikes(i)); % Find window of angles from each swing phase (toe off to heel strike)
       LA_HipAng_Swing.X(i,1:length(TempWindow)) = (TempWindow(1,:));
       LA_HipAng_Swing.Y(i,1:length(TempWindow)) = (TempWindow(2,:));
       LA_HipAng_Swing.Z(i,1:length(TempWindow)) = (TempWindow(3,:));
   end

   LA_HipAng_Stance = struct('X', nan((length(LA_HeelStrikes)-1),300), 'Y', nan((length(LA_HeelStrikes)-1),300), 'Z', nan((length(LA_HeelStrikes)-1),300));
   for i = 1:(length(LA_HeelStrikes)-1)
       TempWindow = LA_HipAng(1:3,LA_HeelStrikes(i):LA_ToeOffs(i+1)); % Find window of angles from each sstance phase (heel strike to toe off)
       LA_HipAng_Stance.X(i,1:length(TempWindow)) = (TempWindow(1,:));
       LA_HipAng_Stance.Y(i,1:length(TempWindow)) = (TempWindow(2,:));
       LA_HipAng_Stance.Z(i,1:length(TempWindow)) = (TempWindow(3,:));
   end
    
   LA_HipAngs = struct('Stance', LA_HipAng_Stance, 'Swing', LA_HipAng_Swing);

%% Make a structure of each segment in all 3 planes for stance and swing for the less affected leg

% Less Affected Femur Segments
   LA_FemurSeg_Swing = struct('X', nan((length(LA_HeelStrikes)-1),300), 'Y', nan((length(LA_HeelStrikes)-1),300), 'Z', nan((length(LA_HeelStrikes)-1),300));
   for i = 1:(length(LA_HeelStrikes)-1)
       TempWindow = LA_FemurSeg(1:3,LA_ToeOffs(i):LA_HeelStrikes(i)); % Find window of angles from each swing phase (toe off to heel strike)
       LA_FemurSeg_Swing.X(i,1:length(TempWindow)) = (TempWindow(1,:));
       LA_FemurSeg_Swing.Y(i,1:length(TempWindow)) = (TempWindow(2,:));
       LA_FemurSeg_Swing.Z(i,1:length(TempWindow)) = (TempWindow(3,:));
   end

   LA_FemurSeg_Stance = struct('X', nan((length(LA_HeelStrikes)-1),300), 'Y', nan((length(LA_HeelStrikes)-1),300), 'Z', nan((length(LA_HeelStrikes)-1),300));
   for i = 1:(length(LA_HeelStrikes)-1)
       TempWindow = LA_FemurSeg(1:3,LA_HeelStrikes(i):LA_ToeOffs(i+1)); % Find window of angles from each sstance phase (heel strike to toe off)
       LA_FemurSeg_Stance.X(i,1:length(TempWindow)) = (TempWindow(1,:));
       LA_FemurSeg_Stance.Y(i,1:length(TempWindow)) = (TempWindow(2,:));
       LA_FemurSeg_Stance.Z(i,1:length(TempWindow)) = (TempWindow(3,:));
   end
    
   LA_FemurSegs = struct('Stance', LA_FemurSeg_Stance, 'Swing', LA_FemurSeg_Swing);

% Less Affected Tibia Segments
   LA_TibSeg_Swing = struct('X', nan((length(LA_HeelStrikes)-1),300), 'Y', nan((length(LA_HeelStrikes)-1),300), 'Z', nan((length(LA_HeelStrikes)-1),300));
   for i = 1:(length(LA_HeelStrikes)-1)
       TempWindow = LA_TibSeg(1:3,LA_ToeOffs(i):LA_HeelStrikes(i)); % Find window of angles from each swing phase (toe off to heel strike)
       LA_TibSeg_Swing.X(i,1:length(TempWindow)) = (TempWindow(1,:));
       LA_TibSeg_Swing.Y(i,1:length(TempWindow)) = (TempWindow(2,:));
       LA_TibSeg_Swing.Z(i,1:length(TempWindow)) = (TempWindow(3,:));
   end

   LA_TibSeg_Stance = struct('X', nan((length(LA_HeelStrikes)-1),300), 'Y', nan((length(LA_HeelStrikes)-1),300), 'Z', nan((length(LA_HeelStrikes)-1),300));
   for i = 1:(length(LA_HeelStrikes)-1)
       TempWindow = LA_TibSeg(1:3,LA_HeelStrikes(i):LA_ToeOffs(i+1)); % Find window of angles from each sstance phase (heel strike to toe off)
       LA_TibSeg_Stance.X(i,1:length(TempWindow)) = (TempWindow(1,:));
       LA_TibSeg_Stance.Y(i,1:length(TempWindow)) = (TempWindow(2,:));
       LA_TibSeg_Stance.Z(i,1:length(TempWindow)) = (TempWindow(3,:));
   end
    
   LA_TibSegs = struct('Stance', LA_TibSeg_Stance, 'Swing', LA_TibSeg_Swing);

 % Less Affected Foot Segments
   LA_FootSeg_Swing = struct('X', nan((length(LA_HeelStrikes)-1),300), 'Y', nan((length(LA_HeelStrikes)-1),300), 'Z', nan((length(LA_HeelStrikes)-1),300));
   for i = 1:(length(LA_HeelStrikes)-1)
       TempWindow = LA_FootSeg(1:3,LA_ToeOffs(i):LA_HeelStrikes(i)); % Find window of angles from each swing phase (toe off to heel strike)
       LA_FootSeg_Swing.X(i,1:length(TempWindow)) = (TempWindow(1,:));
       LA_FootSeg_Swing.Y(i,1:length(TempWindow)) = (TempWindow(2,:));
       LA_FootSeg_Swing.Z(i,1:length(TempWindow)) = (TempWindow(3,:));
   end

   LA_FootSeg_Stance = struct('X', nan((length(LA_HeelStrikes)-1),300), 'Y', nan((length(LA_HeelStrikes)-1),300), 'Z', nan((length(LA_HeelStrikes)-1),300));
   for i = 1:(length(LA_HeelStrikes)-1)
       TempWindow = LA_FootSeg(1:3,LA_HeelStrikes(i):LA_ToeOffs(i+1)); % Find window of angles from each sstance phase (heel strike to toe off)
       LA_FootSeg_Stance.X(i,1:length(TempWindow)) = (TempWindow(1,:));
       LA_FootSeg_Stance.Y(i,1:length(TempWindow)) = (TempWindow(2,:));
       LA_FootSeg_Stance.Z(i,1:length(TempWindow)) = (TempWindow(3,:));
   end
    
   LA_FootSegs = struct('Stance', LA_FootSeg_Stance, 'Swing', LA_FootSeg_Swing);

%% Make a structure of stance force in each plane for the less affected leg
% Use 10 * i because force is collected at 1000 Hz rather than 100 Hz for angles and events - the correct force index will be the angle index * 10
% (e.g. there if there is a heel strike at frame 55 for angles (100 Hz) the same heel strike will occur at frame 550 for forces (1000 Hz)

   LA_Force_Stance = struct('X', nan((length(LA_HeelStrikes)-1),3000), 'Y', nan((length(LA_HeelStrikes)-1),3000), 'Z', nan((length(LA_HeelStrikes)-1),3000));
   for i = 1:(length(LA_HeelStrikes)-1)
       TempWindow = LA_Force(1:3,(10*(LA_HeelStrikes(i))):(10*(LA_ToeOffs(i+1)))); % Find window of forces from each stance phase (heel strike to toe off)
       LA_Force_Stance.X(i,1:length(TempWindow)) = (TempWindow(1,:)); % Propoulsion/braking forces
       LA_Force_Stance.Y(i,1:length(TempWindow)) = (TempWindow(2,:)); % Medial/lateral forces
       LA_Force_Stance.Z(i,1:length(TempWindow)) = (TempWindow(3,:)); % Vertical forces
   end

   LA_Forces = struct('Stance', LA_Force_Stance);

%% Create less affected leg structure
    Less_Affected = struct('Hip_Angles', LA_HipAngs, 'Knee_Angles', LA_KneeAngs, 'Ankle_Angles', LA_AnkleAngs, 'Femur_Segment', LA_FemurSegs, 'Tibia_Segment', LA_TibSegs, 'Foot_Segment', LA_FootSegs, 'Forces', LA_Forces);

%% Create phase structure
    PhaseStruct = struct('More_Affected', More_Affected, 'Less_Affected', Less_Affected);

%% Create interpolated phase structure using custom function (see end of script)

    % Determine the maximum number of columns
        MaxColumns = 100; % Change this to your desired maximum number of columns
    % Interpolate the nested structure
        PhaseStructInterp = InterpolateNestedStruct(PhaseStruct, MaxColumns);

%% Create or add to full data structure

ExportData = inputdlg('Save and Export to MAT files? Y/N: ');

    if strcmp(ExportData, 'Y')
    MatFilePath = fullfile(RootPath, 'GaitCycle_Data.mat'); % Use fullfile for path construction
    MatFilePathInterp = fullfile(RootPath, 'GaitCycle_Data_Interpolated.mat'); % Use fullfile for path construction

         % Load existing data or initialize GaitCycle_Data
         % As structure gets big this will take a LONG time... May want to do this in sections or adjust the script so it keeps the variables in the
         % workspace
         
        if exist(MatFilePath, 'file') == 2
        load(MatFilePath, 'GaitCycle_Data');
        else
        GaitCycle_Data = struct(); % Initialize GaitCycle_Data if the file doesn't exist
        end

        if exist(MatFilePathInterp, 'file') == 2
        load(MatFilePathInterp, 'GaitCycle_Data_Interpolated');
        else
        GaitCycle_Data_Interpolated = struct(); % Initialize GaitCycle_Data_Interpolated if the file doesn't exist
        end

        % Add PhaseStruct to GaitCycle_Data using SubNum  and TrialName as field names
        GaitCycle_Data.(SubNum).(TrialName) = PhaseStruct;
        GaitCycle_Data = orderfields(GaitCycle_Data);
        GaitCycle_Data_Interpolated.(SubNum).(TrialName) = PhaseStructInterp;
        GaitCycle_Data_Interpolated = orderfields(GaitCycle_Data_Interpolated);


        % Save All_Data to the MAT file
         save(MatFilePath, 'GaitCycle_Data');
         save(MatFilePathInterp, 'GaitCycle_Data_Interpolated');
    
        disp('Data exported and saved successfully. :)');
    else
        disp('Data export cancelled. :(');
    end


%% Function to interpolate data in a structure

function InterpolatedStruct = InterpolateNestedStruct(RawStruct, MaxColumns)
    FieldNames = fieldnames(RawStruct);
    InterpolatedStruct = struct();
    
    for i = 1:numel(FieldNames)
        FieldName = FieldNames{i};
        FieldValue = RawStruct.(FieldName);
        
        if isstruct(FieldValue)
            % If the field is a structure, recursively interpolate it
            InterpolatedStruct.(FieldName) = InterpolateNestedStruct(FieldValue, MaxColumns);
        else
             % If the field is numeric data, interpolate each row individually
            [NumRows, ~] = size(FieldValue);
            InterpolatedData = nan(NumRows, MaxColumns);
            
            for Row = 1:NumRows
                CurrentRowData = FieldValue(Row, :);
                NaNIndices = isnan(CurrentRowData);
                NonNaNIndices = ~NaNIndices;
                
                if sum(NonNaNIndices) > 0
                    % Interpolate or extrapolate only the non-NaN values
                    X = find(NonNaNIndices);
                    XInterpolated = linspace(X(1), X(end), MaxColumns);
                    InterpolatedRowData = interp1(X, CurrentRowData(NonNaNIndices), XInterpolated, 'linear', 'extrap');
                    InterpolatedData(Row, :) = InterpolatedRowData;
                else
                    % If all values are NaN, keep them as NaN
                    InterpolatedData(Row, :) = NaN;
                end
            end
            InterpolatedStruct.(FieldName) = InterpolatedData;
        end
    end
end



