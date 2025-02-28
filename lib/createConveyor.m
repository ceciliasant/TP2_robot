function createConveyor(LTF, HT, WTS, translation)
    % Create a conveyor belt with given dimensions, translation, and color
    
    % Generate points and faces for the conveyor
    [points, faces] = tapete(LTF, HT, WTS);
    % Apply translation to the points
    points = translate(points, translation(1), translation(2), translation(3));
    % Draw the patch
    hold on;
    patch('Vertices', points, 'Faces', faces, ...
          'FaceColor', '#494848', 'EdgeColor', 'black');
end
