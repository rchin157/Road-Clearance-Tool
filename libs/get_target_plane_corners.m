% Get the corners of the observer plane give the forward normal, leftward normal... 
% (90 deg. CCW along horizontal plane from forward), the upward normal (90 deg. CCW along vertical plane...
% from forward) and the dimensions provided (initially set at the averagae car windshield size)
function [c1, c2, c3, c4] = get_target_plane_corners(observer, forward, leftward, observer_width, dir, maxtop, maxside, scantiles)
    % initialize corner lists
    c1 = zeros(scantiles,3);
    c2 = zeros(scantiles,3);
    c3 = zeros(scantiles,3);
    c4 = zeros(scantiles,3);
    % create requested targets
    for i = 1:scantiles
        if dir == "up"
            c1(i,:) = observer(i,:) + [leftward(1) leftward(2) 0].*observer_width*2 + [forward(1) forward(2) 0].*observer_width*2 + [0 0 1].*maxtop;
            c2(i,:) = observer(i,:) + [leftward(1) leftward(2) 0].*observer_width*2 - [forward(1) forward(2) 0].*observer_width*2 + [0 0 1].*maxtop;
            c3(i,:) = observer(i,:) - [leftward(1) leftward(2) 0].*observer_width*2 - [forward(1) forward(2) 0].*observer_width*2 + [0 0 1].*maxtop;
            c4(i,:) = observer(i,:) - [leftward(1) leftward(2) 0].*observer_width*2 + [forward(1) forward(2) 0].*observer_width*2 + [0 0 1].*maxtop;
        elseif dir == "right"
            c1(i,:) = observer(i,:) + [forward(1) forward(2) 0].*observer_width*2 + [0 0 1].*observer_width*2 - [leftward(1) leftward(2) 0].*maxside;
            c2(i,:) = observer(i,:) + [forward(1) forward(2) 0].*observer_width*2 - [0 0 1].*observer_width*2 - [leftward(1) leftward(2) 0].*maxside;
            c3(i,:) = observer(i,:) - [forward(1) forward(2) 0].*observer_width*2 - [0 0 1].*observer_width*2 - [leftward(1) leftward(2) 0].*maxside;
            c4(i,:) = observer(i,:) - [forward(1) forward(2) 0].*observer_width*2 + [0 0 1].*observer_width*2 - [leftward(1) leftward(2) 0].*maxside;
        elseif dir == "left"
            c1(i,:) = observer(i,:) + [forward(1) forward(2) 0].*observer_width*2 + [0 0 1].*observer_width*2 + [leftward(1) leftward(2) 0].*maxside;
            c2(i,:) = observer(i,:) + [forward(1) forward(2) 0].*observer_width*2 - [0 0 1].*observer_width*2 + [leftward(1) leftward(2) 0].*maxside;
            c3(i,:) = observer(i,:) - [forward(1) forward(2) 0].*observer_width*2 - [0 0 1].*observer_width*2 + [leftward(1) leftward(2) 0].*maxside;
            c4(i,:) = observer(i,:) - [forward(1) forward(2) 0].*observer_width*2 + [0 0 1].*observer_width*2 + [leftward(1) leftward(2) 0].*maxside;
        else
            error("side or top must be specified for observer planes")
        end
    end
end