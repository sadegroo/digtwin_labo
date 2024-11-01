function [ Tfull,Tparts,Tcumul ] = DH_full( DH_table )
% Compute DH-transformations: takes every row of DH-table and computes transformation matrices
% According to the classic Denavit-Hartenberg method.
% Arguments:
%   DH_table: a n x 4 double with columns in the order: alpha - a - d - theta
%       alpha and theta should be entered in RADIANS!
% Return Values:
%   Tfull: Returns a 4x4 double DH-transform of the full transform from frame 0 to the
%   last frame
%   Tparts: Returns a cell array with the 4x4 double DH transforms for each row of the DH table 
%       cell i contains T(i-1)->i. To address matrix, type >> Tparts{i}
%   Tcumul: Returns a cell array of the intermediate transforms from frame 0
%       to the frame corresponding to the respective row
%       cell i contains T0->i. To address matrix, type >> Tcumu{i}    

% initialize
temp = eye(4);
sz = size(DH_table,1);
Tparts = cell(1,sz);
Tcumul = cell(1,sz);

for i = 1:sz    
% address a line of the DH table
alpha = DH_table(i,1);
a = DH_table(i,2);
d = DH_table(i,3);
theta = DH_table(i,4);

% Generate DH matrix
Tparts{i} = [cos(theta)  -sin(theta)*cos(alpha)   sin(theta)*sin(alpha)  a*cos(theta) ;
             sin(theta)   cos(theta)*cos(alpha)  -cos(theta)*sin(alpha)  a*sin(theta) ;
             0            sin(alpha)              cos(alpha)             d            ;
             0            0                       0                      1            ];
            
%right multiply with new DH matrix          
temp = temp*Tparts{i};

% Return intermediate transform up to frame "i"
Tcumul{i} = temp;
end

% Return full DH matrix
Tfull = temp;

end