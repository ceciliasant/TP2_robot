function h = DrawLinks(Org)
X = Org(1,:);
Y = Org(2,:);
Z = Org(3,:);
h = plot3(X, Y, Z);
h.LineWidth = 1;
h.Marker = 'o';
h.MarkerSize = 2;

% for i = 1:size(Org,2)
%     h = plot3(Org[i,:])
% end