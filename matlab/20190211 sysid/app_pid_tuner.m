%% cleaning
clear all

%% save
% IdentifyedModelDutyToTranslationalVelocity = Plant1;
% save IdentifyedModelDutyToTranslationalVelocity IdentifyedModelDutyToTranslationalVelocity

% IdentifyedModelDutyToRotationalVelocity = Plant1;
% save IdentifyedModelDutyToRotationalVelocity IdentifyedModelDutyToRotationalVelocity

%% translational velocity
load('IdentifyedModelDutyToTranslationalVelocity.mat');
ModelTrans = c2d(tf(IdentifyedModelDutyToTranslationalVelocity), 0.001);
load('ControllerTrans.mat');

%% trans PID Tuner
pidTuner(ModelTrans);

%% save Trans Controller
% ControllerTrans = C;
% save ControllerTrans ControllerTrans;

%% rotational velocity
load('IdentifyedModelDutyToRotationalVelocity.mat');
ModelRot = c2d(tf(IdentifyedModelDutyToRotationalVelocity), 0.001);
load('ControllerRot.mat');

%% rot PID Tuner
pidTuner(ModelRot);

%% save Trans Controller
% ControllerRot = C;
% save ControllerRot ControllerRot;
