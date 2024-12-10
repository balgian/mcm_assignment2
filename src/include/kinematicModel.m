%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        J % Jacobian
    end

    methods
        % Constructor to initialize the geomModel property
        function self = kinematicModel(gm)
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, self.gm.jointNumber);
            else
                error('Not enough input arguments (geometricModel)')
            end
        end
        function updateJacobian(self)
        %% Update Jacobian function
        % The function update:
        % - J: end-effector jacobian matrix

            % TO DO
            %% Compute the simple Jacobian
            jJb_A = zeros(3, self.gm.jointNumber);
            jJb_L = [zeros(2, self.gm.jointNumber); 
                ones(1,self.gm.jointNumber)];
            for j = 1:self.gm.jointNumber
                if ~self.gm.jointType(j)
                    jJb_A(:, j) = [0; 0; 1];

                    bTj = self.gm.getTransformWrtBase(j);
                    b_r_j_4 = bTj * [0; 0; 0; 1];
                    b_r_j = b_r_j_4(1:end-1);
                    jJb_L(:, j) = cross([0; 0; 1], b_r_j);
                end
            end
            self.J = [jJb_A; jJb_L];
        end
    end
end

