run RRpendulum_forkin_dyn_noimage.m
run RRpendulum_Parameters_num_BLDC.m
run RRpendulum_FSFB_controldesign_torque.m
run RRpendulum_UKF_design.m
run RRpendulum_swingup_controldesign.m
run RRpendulum_axis1acc_controldesign.m 

clear
if isempty(matlab.project.rootProject)
    openProject('C:/Users/u0130154/MATLAB/projects/digtwin_labo/digtwin_labo.prj');
end
prj = matlab.project.rootProject;
data_dir = fullfile(prj.RootFolder, 'data');

load(fullfile(data_dir, 'axis1acc_design.mat'))
load(fullfile(data_dir, 'FSFB_torque_design.mat'))
load(fullfile(data_dir, 'RRpendulum_EOM.mat'))
load(fullfile(data_dir, 'RRpendulum_params_BLDC.mat'))
load(fullfile(data_dir, 'swingup_design.mat'))
load(fullfile(data_dir, 'UKF_design.mat'))
fsf_par = design.par;
ukf_par = ukf.par;
swingup_par = swingup.par;
axis1acc_par = axis1acc.par;

clear data_dir prj