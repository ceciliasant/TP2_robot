classdef Block3D
    properties
        VerticesMatrix  % Homogeneous coordinates for block vertices
        FacesMatrix     % Matrix of the vertices that make each face of the block
        PatchHandle     % Handle for the graphical patch object
    end
    
    methods
        function obj = Block3D(blockLength, blockWidth, blockHeight, transformMatrix, blockColor, alpha)
            % BLOCK3D: Constructs a rectangular block and visualizes it
            % Inputs:
            %   transformMatrix: 4x4 transformation matrix
            %   blockLength: Length of the block along the y-axis
            %   blockWidth: Width of the block along the x-axis
            %   blockHeight: Height of the block along the z-axis
            %   blockColor: Color of the block 
            %   alpha: Transparency (optional, default opaque)
            
            if nargin < 6
                alpha = 1;  % Default opaque
            end
          
            obj.VerticesMatrix = [
                 blockWidth/2, -blockLength/2, 0;    % point 1
                 blockWidth/2,  blockLength/2, 0;    % point 2
                -blockWidth/2,  blockLength/2, 0;    % point 3
                -blockWidth/2, -blockLength/2, 0;    % point 4
                 blockWidth/2, -blockLength/2, blockHeight;  % point 5
                 blockWidth/2,  blockLength/2, blockHeight;  % point 6
                -blockWidth/2,  blockLength/2, blockHeight;  % point 7
                -blockWidth/2, -blockLength/2, blockHeight;  % point 8
            ];

            % homogenous coordinates
            obj.VerticesMatrix = [obj.VerticesMatrix';ones(1, size(obj.VerticesMatrix', 2))]';
            
            transformedVertices = transformMatrix * obj.VerticesMatrix';

            % block faces
            obj.FacesMatrix = [
                1 2 3 4;  % Bottom face
                5 6 7 8;  % Top face
                1 2 6 5;  % Front face
                2 3 7 6;  % Right face
                3 4 8 7;  % Back face
                4 1 5 8   % Left face
            ];
            
            % Patch draw
            obj.PatchHandle = patch('Vertices', transformedVertices(1:3, :)', ...
                                    'Faces', obj.FacesMatrix, ...
                                    'FaceColor', blockColor, ...
                                    'FaceAlpha', alpha);
        end
        
        function obj = updatePosition(obj, transformMatrix)
            % UPDATEPOSITION: Updates the block's position with a new transformation
            % Inputs:
            %   transformMatrix: 4x4 transformation matrix

            updatedVertices = transformMatrix * obj.VerticesMatrix';
            obj.PatchHandle.Vertices = updatedVertices(1:3, :)';
        end
    end
end
