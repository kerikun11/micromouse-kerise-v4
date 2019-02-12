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

%% trans PID Tuner
pidTuner(ModelTrans);

%% rotational velocity
load('IdentifyedModelDutyToRotationalVelocity.mat');
ModelRot = c2d(tf(IdentifyedModelDutyToRotationalVelocity), 0.001);

%% rot PID Tuner
pidTuner(ModelRot);
