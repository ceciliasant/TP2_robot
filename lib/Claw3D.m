classdef Claw3D
    properties
        VerticesMatrix  % Homogeneous coordinates for claw vertices
        FacesMatrix     % Matrix of the vertices that make each face of the claw
        PatchHandle     % Handle for the graphical patch object
    end
    
    methods
        function obj = Claw3D(blockLength, blockWidth,blockHeight, transformMatrix,clawColor, alpha)
            % CLAW3D: Constructs a rectangular claw and visualizes it
            % Inputs:
            %   transformMatrix: 4x4 transformation matrix
            %   clawLength: Length of the claw along the y-axis
            %   clawWidth: Width of the claw along the x-axis
            %   clawHeight: Height of the claw along the z-axis
            %   clawColor: Color of the claw
            %   alpha: Transparency (optional, default opaque)
            
            if nargin < 6
                alpha = 1;  % Default opaque
            end
          
            obj.VerticesMatrix = [
                blockWidth/2, -blockLength/2, 0; % point 1
                blockWidth/2, blockLength/2, 0; % point 2
                -blockWidth/2, blockLength/2, 0; % point 3
                -blockWidth/2, -blockLength/2, 0; % point 4
                blockWidth/2, -blockLength/2, blockHeight; % point 1
                blockWidth/2, blockLength/2, blockHeight; % point 2
                -blockWidth/2, blockLength/2, blockHeight; % point 3
                -blockWidth/2, -blockLength/2, blockHeight; % point 4
                ];

            % homogenous coordinates
            obj.VerticesMatrix = [obj.VerticesMatrix';ones(1, size(obj.VerticesMatrix', 2))]';
            
            transformedVertices = transformMatrix * obj.VerticesMatrix';

            % block faces
            obj.FacesMatrix = [
                1 2 6 5;  % Front face
                1 2 3 4;  % Left face
                4 3 7 8   % Back face
            ];
            
            % Patch draw
            obj.PatchHandle = patch('Vertices', transformedVertices(1:3, :)', ...
                                    'Faces', obj.FacesMatrix, ...
                                    'FaceColor', clawColor, ...
                                    'FaceAlpha', alpha);
        end
        
        function obj = updatePosition(obj, transformMatrix)
            % UPDATEPOSITION: Updates the claw's position with a new transformation
            % Inputs:
            %   transformMatrix: 4x4 transformation matrix

            updatedVertices = transformMatrix * obj.VerticesMatrix';
            obj.PatchHandle.Vertices = updatedVertices(1:3, :)';
        end
    end
end
