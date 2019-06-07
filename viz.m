function handle = viz(qhat1, qhat2, q, a, margin, frame, handle)
%VIZ Simulation visualization and drawing tools

lowerbound = (1-margin)*9.80665;
upperbound = (1+margin)*9.80665;
if lowerbound < norm(a) && norm(a) < upperbound
    c = [0.4940, 0.1840, 0.5560];
else
    c = [0.8150, 0.0700, 0.0840];
end

if isempty(handle)
    subplot(121); hold on; title('Pitch');
    drawEnvironment(frame, 'ZX');
    handle{1} = drawVector(Q(q).toRotm()*a, frame, [], c);
    handle{2} = drawVector(Q(q).toRotm()*[0;0;10], frame, []);
    handle{3} = drawVector(Q(qhat1).toRotm()*[0;0;7], frame, []);
    handle{4} = drawVector(Q(qhat2).toRotm()*[0;0;5], frame, []);
    
    subplot(122); hold on; title('Roll');
    drawEnvironment(frame, 'ZY');
    handle{11} = drawVector(Q(q).toRotm()*a, frame, [], c);
    handle{12} = drawVector(Q(q).toRotm()*[0;0;10], frame, []);
    handle{13} = drawVector(Q(qhat1).toRotm()*[0;0;7], frame, []);
    handle{14} = drawVector(Q(qhat2).toRotm()*[0;0;5], frame, []);

else
    handle{1} = drawVector(Q(q).toRotm()*a, frame, handle{1}, c);
    handle{2} = drawVector(Q(q).toRotm()*[0;0;10], frame, handle{2});
    handle{3} = drawVector(Q(qhat1).toRotm()*[0;0;7], frame, handle{3});
    handle{4} = drawVector(Q(qhat2).toRotm()*[0;0;5], frame, handle{4});
    
    handle{11} = drawVector(Q(q).toRotm()*a, frame, handle{11}, c);
    handle{12} = drawVector(Q(q).toRotm()*[0;0;10], frame, handle{12});
    handle{13} = drawVector(Q(qhat1).toRotm()*[0;0;7], frame, handle{13});
    handle{14} = drawVector(Q(qhat2).toRotm()*[0;0;5], frame, handle{14});
end

end


function drawEnvironment(frame, plane)
    axis equal; grid on;
    axis([-1 1 -1 1 -1 1]*16)
    xlabel('X'); ylabel('Y'); zlabel('Z');    
    
    % draw coordinate axes
    view(0,0); % hack to make it 3d (for axis)
    A = axis;
    k = 6.4;
    
    % figure out frame
    if strcmp(frame,'FLU')
        R = eye(3);
        O = [A(1);A(3);A(5)];
    elseif strcmp(frame,'FRD')
        R = [1 0 0; 0 -1 0; 0 0 -1];
        O = [A(1);-A(3);-A(5)];
    end
    
    drawCoordinateAxes(O, R, k, 1);
    
    if strcmp(plane,'ZX'), view(0,0);
    elseif strcmp(plane,'ZY'), view(90,0); end
end


function handle = drawVector(v, frame, handle, varargin)
    if nargin == 4, c = varargin{1}; else, c = []; end
    
    % figure out frame
    if strcmp(frame,'FLU'), R = eye(3);
    elseif strcmp(frame,'FRD'), R = [1 0 0; 0 -1 0; 0 0 -1]; end
    
    v = R*v;

    XX = [0 v(1)];
    YY = [0 v(2)];
    ZZ = [0 v(3)];
    if isempty(handle)
        handle = plot3(XX,YY,ZZ,'LineWidth',2);
    else
        set(handle,'XData',XX,'YData',YY,'ZData',ZZ);
    end
    
    if ~isempty(c), set(handle,'Color',c); end
end

function handle = drawCoordinateAxes(O, R, k, alpha, varargin)
%PLOTCOORDINATEFRAME Plot a coordinate frame origin and orientation
%   Coordinate frames have a origin and an orientation. This function draws
%   the coordinate axes in a common frame.

    % k     Size of each axis
    
    if length(varargin)==0 || isempty(varargin{1}), handle = {}; end
    if length(varargin)>=1, handle = varargin{1}; end

    % Create coordinate axes starting at 0
    kk = linspace(0,k,100);
    CX = [kk; zeros(1,length(kk)); zeros(1,length(kk))];
    CY = [zeros(1,length(kk)); kk; zeros(1,length(kk))];
    CZ = [zeros(1,length(kk)); zeros(1,length(kk)); kk];
    
    % First rotate the coordinate frame from the local frame to the
    % orientation of the desired frame in which we want to plot.
    CX = R'*CX;
    CY = R'*CY;
    CZ = R'*CZ;
    
    % Then translate this frame to its origin
    CX = repmat(O, 1, size(CX,2)) + CX;
    CY = repmat(O, 1, size(CY,2)) + CY;
    CZ = repmat(O, 1, size(CZ,2)) + CZ;
    
    % Plot the axes
    ls = '-';
    if isempty(handle)
        handle{1} = plot3(CX(1,:), CX(2,:), CX(3,:),'color','r','linewidth',2,'linestyle',ls);
        handle{2} = plot3(CY(1,:), CY(2,:), CY(3,:),'color','g','linewidth',2,'linestyle',ls);
        handle{3} = plot3(CZ(1,:), CZ(2,:), CZ(3,:),'color','b','linewidth',2,'linestyle',ls);
        
        handle{1}.Color(4) = alpha;
        handle{2}.Color(4) = alpha;
        handle{3}.Color(4) = alpha;
    else
        set(handle{1},'XData',CX(1,:),'YData',CX(2,:),'ZData',CX(3,:));
        set(handle{2},'XData',CY(1,:),'YData',CY(2,:),'ZData',CY(3,:));
        set(handle{3},'XData',CZ(1,:),'YData',CZ(2,:),'ZData',CZ(3,:));
    end
end

