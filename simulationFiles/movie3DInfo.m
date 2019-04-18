function movie3DInfo(step, allStep, figNum, step1st, stepLast)
% This function generates a 3D biped movie using step and allStep.
% q = [qzT; qyT; qxT; q1R; q2R; q3R; q1L; q2L; q3L];
% BAG20141119
% BAG20141204 Updated to make movies and operate in realtime when not
% making a movie.

if (nargin<4); step1st=1; stepLast=allStep.stepsComplete; end

% Initialize variables
drawRate = 10; bigScreen=1; movie=0; movieName='tempMovie';
dataStepTime = step(step1st).t(2) - step(step1st).t(1);
if movie; drawRate = floor((1/30)/dataStepTime); end

% Get terrain coordinates
terrainCoordinates = zeros(3,allStep.stepsComplete);
for n=1:allStep.stepsComplete
terrainCoordinates(:,n) = step(n).stanceFoot;
end

% Initialization
stanceFoot = step(step1st).stanceFoot; leg = step(step1st).leg;
x=step(step1st).x(:,1); q=x(1:9); dq=x(10:18);
if leg;
    [pcm p0 p0T pHR p1R p2R p3R p4R pHL p1L p2L p3L p4L] = ...
              Cfcn_ATRIAS3D_Primary_Points_Anim_StanceLeft([q;stanceFoot]);
else; [pcm p0 p0T pHR p1R p2R p3R p4R pHL p1L p2L p3L p4L] = ...
              Cfcn_ATRIAS3D_Primary_Points_Anim_StanceLeft([q;stanceFoot]);
      [pcm p0 p0T pHR p1R p2R p3R p4R pHL p1L p2L p3L p4L] = ...
              Cfcn_ATRIAS3D_Primary_Points_Anim_StanceLeft(...
              [q;2*stanceFoot-p4R]); end
   
%--------------------------------------------------------------------------
% Visualization Setup
%--------------------------------------------------------------------------
Maize = [1 0.7961 0.01961];
Blue = [0 0.1529 0.2980];
leg1Color= Blue;% Official umich Maize.
leg2Color= Blue; % Maize
torsoColor= Maize; % Official umich Blue.
groundColor=[0 0.4 0];

% Collect Axis Info for Movie
    maxScreenY = max(terrainCoordinates(2,:))+2;
    minScreenY = min(terrainCoordinates(2,:))-2; % Modified for trials.
    maxScreenX = max(terrainCoordinates(1,:))+2;
    minScreenX = min(terrainCoordinates(1,:))-2;
    minScreenZ = min(terrainCoordinates(3,:))-0.25;
    maxScreenZ = 0.6*(maxScreenY-minScreenY)-minScreenZ;
    yRange = maxScreenY - minScreenY; xRange = maxScreenX - minScreenX;
    delta = abs(yRange - xRange)/2;
    if ( yRange > xRange)
        minScreenX = minScreenX - delta; maxScreenX = maxScreenX + delta;
    else
        minScreenY = minScreenY - delta; maxScreenY = maxScreenY + delta;
    end

    % Initialize Drawing
    AZ=[75;0;0;75]; EL=[10,0,90,30]; % Planar-back % Rear View % Top View
    figure(figNum); 
    mf = figure(figNum); mn=1; % mf and mn used for movie if applicable.
    for i=1:4 % Generate three figures (or less depending on i)
    subplot(2,2,i)
    title(sprintf('Quick View 3D %i',(figNum)))
    cla; plot3(0,0,0); grid on; axis([-1 1 -1 1 0 2 0 1]);
    lineData(i).axis = gca;
    view(AZ(i),EL(i)); 
    
    % Define stance leg
    lineData(i).p1HR=line([p1R(1) pHR(1)],[p1R(2) pHR(2)],[p1R(3) pHR(3)]);
    lineData(i).p2HR=line([p2R(1) pHR(1)],[p2R(2) pHR(2)],[p2R(3) pHR(3)]);
    lineData(i).p31R=line([p3R(1) p1R(1)],[p3R(2) p1R(2)],[p3R(3) p1R(3)]);
    lineData(i).p42R=line([p4R(1) p2R(1)],[p4R(2) p2R(2)],[p4R(3) p2R(3)]);
    lineData(i).pHR0TR=line([p0(1) pHR(1)],[p0(2) pHR(2)],[p0(3) pHR(3)]);
    
    legThickness = 5; if i==4; legThickness = 2; end
    hipThickness = 8; if i==4; hipThickness = 3; end
    set(lineData(i).p1HR,'LineWidth',legThickness,'Color',leg1Color); lineData(i).p1HR.Color(4) = 0;
    set(lineData(i).p2HR,'LineWidth',legThickness,'Color',leg1Color); 
    set(lineData(i).p31R,'LineWidth',legThickness,'Color',leg1Color); lineData(i).p31R.Color(4) = 0;
    set(lineData(i).p42R,'LineWidth',legThickness,'Color',leg1Color);
    set(lineData(i).pHR0TR,'LineWidth',hipThickness,'Color',torsoColor);
    
    % Define swing leg
    lineData(i).p1HL=line([p1L(1) pHL(1)],[p1L(2) pHL(2)],[p1L(3) pHL(3)]); 
    lineData(i).p2HL=line([p2L(1) pHL(1)],[p2L(2) pHL(2)],[p2L(3) pHL(3)]);
    lineData(i).p31L=line([p3L(1) p1L(1)],[p3L(2) p1L(2)],[p3L(3) p1L(3)]); 
    lineData(i).p42L=line([p4L(1) p2L(1)],[p4L(2) p2L(2)],[p4L(3) p2L(3)]);
    lineData(i).pHR0TL=line([p0(1) pHL(1)],[p0(2) pHL(2)],[p0(3) pHL(3)]);
    
    set(lineData(i).p1HL,'LineWidth',legThickness,'Color',leg2Color); lineData(i).p1HL.Color(4) = 0;
    set(lineData(i).p2HL,'LineWidth',legThickness,'Color',leg2Color);
    set(lineData(i).p31L,'LineWidth',legThickness,'Color',leg2Color); lineData(i).p31L.Color(4) = 0;
    set(lineData(i).p42L,'LineWidth',legThickness,'Color',leg2Color);
    set(lineData(i).pHR0TL,'LineWidth',hipThickness,'Color',torsoColor);
    
    % Define torso
    torsoThickness = 8; if i==4; torsoThickness = 3; end
    lineData(i).torso=line([p0(1) p0T(1)],[p0(2) p0T(2)],[p0(3) p0T(3)]);
    set(lineData(i).torso,'LineWidth',torsoThickness,'Color',torsoColor);
    
    % Define Terrain
    terrainProfile=line(terrainCoordinates(1,:),terrainCoordinates(2,:),...
        terrainCoordinates(3,:));
    terrainThickness = 20; if i==4; terrainThickness = 20; end
    set(terrainProfile,'LineWidth',15,'Color',groundColor,...
        'LineStyle','none','Marker','.','MarkerSize',terrainThickness);
       
    % Set additional data for top left
    if i==3
       % Define text
        lineData(i).height=text(pcm(1)+0.5,pcm(2)+0.5,pcm(3)+0.5,...
        sprintf('%0.3g cm height/step',step(step1st).stepHeight*100));
        lineData(i).velocity=text(pcm(1)+0.5,pcm(2)+0.5,pcm(3)+0.5,...
        sprintf('[%0.2g, %0.2g, %0.2g] m/s',step(step1st).avgCoMVelocity));
        lineData(i).force=text(pcm(1)-0.5,pcm(2)-0.5,pcm(3)+0.5, ...
        sprintf('%0.3g N',step(step1st).forceVector(1)));
        lineData(i).time=text(pcm(1)-0.5,pcm(2)-0.5,pcm(3)+0.5, ...
        sprintf('%0.3g s',step(step1st).t(1)));
        set(lineData(i).height,'FontWeight','bold')
        set(lineData(i).velocity,'FontWeight','bold')
        set(lineData(i).force,'FontWeight','bold')
        set(lineData(i).time,'FontWeight','bold')
    else    
    
    % Set permanent axis for big screen
    if i==4
       axis([minScreenX maxScreenX minScreenY maxScreenY ...
           minScreenZ maxScreenZ]) 
    else
    % Define text
    lineData(i).velocity=text(pcm(1)+0.5,pcm(2)+0.5,pcm(3)+0.5,...
        sprintf('%0.3g m/s',step(step1st).avgCoMVelocity));
    set(lineData(i).velocity,'FontWeight','bold')
    end
    end
    
    end
%--------------------------------------------------------------------------


%--------------------------------------------------------------------------
% Make movie
%--------------------------------------------------------------------------
for n=step1st:stepLast
    % Set variables for current step.
    stanceFoot = step(n).stanceFoot; leg = step(n).leg; 
    j=1; jMax = size(step(n).t,1); drawNum = 0; 
    stepVelocity = step(n).avgCoMVelocity;
    if (abs(stepVelocity(3))<0.0001); stepVelocity(3) = 0; end
while (j<jMax)
    % Find current coordinates.
    x=step(n).x(:,j); q=x(1:9); dq=x(10:18);
    if leg;
    [pcm p0 p0T pHR p1R p2R p3R p4R pHL p1L p2L p3L p4L] = ...
              Cfcn_ATRIAS3D_Primary_Points_Anim_StanceLeft([q;stanceFoot]);
    else; [pcm p0 p0T pHR p1R p2R p3R p4R pHL p1L p2L p3L p4L] = ...
                  Cfcn_ATRIAS3D_Primary_Points_Anim_StanceLeft([q;stanceFoot]);
          [pcm p0 p0T pHR p1R p2R p3R p4R pHL p1L p2L p3L p4L] = ...
                  Cfcn_ATRIAS3D_Primary_Points_Anim_StanceLeft(...
                  [q;2*stanceFoot-p4R]); end
          
        % Update figure data if already defined.
        for i=1:4
        % for i=1 % Speed up visualization.
        set(lineData(i).p1HR,'XData',[p1R(1) pHR(1)],'YData',[p1R(2) pHR(2)],'ZData',[p1R(3) pHR(3)]);
        set(lineData(i).p2HR,'XData',[p2R(1) pHR(1)],'YData',[p2R(2) pHR(2)],'ZData',[p2R(3) pHR(3)]);
        set(lineData(i).p31R,'XData',[p3R(1) p1R(1)],'YData',[p3R(2) p1R(2)],'ZData',[p3R(3) p1R(3)]);
        set(lineData(i).p42R,'XData',[p4R(1) p2R(1)],'YData',[p4R(2) p2R(2)],'ZData',[p4R(3) p2R(3)]);
        set(lineData(i).pHR0TR,'XData',[p0(1) pHR(1)],'YData',[p0(2) pHR(2)],'ZData',[p0(3) pHR(3)]);
        set(lineData(i).p1HL,'XData',[p1L(1) pHL(1)],'YData',[p1L(2) pHL(2)],'ZData',[p1L(3) pHL(3)]);
        set(lineData(i).p2HL,'XData',[p2L(1) pHL(1)],'YData',[p2L(2) pHL(2)],'ZData',[p2L(3) pHL(3)]);
        set(lineData(i).p31L,'XData',[p3L(1) p1L(1)],'YData',[p3L(2) p1L(2)],'ZData',[p3L(3) p1L(3)]);
        set(lineData(i).p42L,'XData',[p4L(1) p2L(1)],'YData',[p4L(2) p2L(2)],'ZData',[p4L(3) p2L(3)]);
        set(lineData(i).pHR0TL,'XData',[p0(1) pHL(1)],'YData',[p0(2) pHL(2)],'ZData',[p0(3) pHL(3)]);
        set(lineData(i).torso,'XData',[p0(1) p0T(1)],'YData',[p0(2) p0T(2)],'ZData',[p0(3) p0T(3)]);
        set(lineData(i).velocity,'Position',[pcm(1)+0.5,pcm(2)+0.5,pcm(3)+0.5],'String',sprintf('[%0.2g, %0.2g, %0.2g] m/s',stepVelocity));
        % Reset axis.
        if ~(i==4)
        
        if 1 % Note, moving axis slows process further in simulation.
        set(lineData(i).axis,'XLim',[([-1 1]+pcm(1))],'YLim',[([-1 1]+pcm(2))],...
            'ZLim',([-1.25 0.75]+pcm(3))); % Fixed ZLim.
        elseif 0; set(gca,'XLim',[([-1 1]+pcm(1))],'YLim',[([-1 1]+pcm(2))],...
            'ZLim',[([-1.2 0.8]+pcm(3))]); end
        end; 
        
        % Added text for top left corner
        if i==3
            if (abs(step(n).stepHeight)<0.0001)
                step(n).stepHeight = 0; 
                set(lineData(i).height,'Position',[pcm(1)-1+0.1,pcm(2)+0.5+0.4,pcm(3)+0.1],'String',sprintf(''));
            else
                set(lineData(i).height,'Position',[pcm(1)-1+0.1,pcm(2)+0.5+0.4,pcm(3)+0.1],'String',sprintf('%0.3g cm step height',step(n).stepHeight*100));
            end
            
            if (abs(step(n).forceVector(1))<0.0001)
                set(lineData(i).force,'Position',[pcm(1)-1+0.1,pcm(2)-0.3,pcm(3)-0.3],'String',sprintf(''));
            else
                set(lineData(i).force,'Position',[pcm(1)-1+0.1,pcm(2)-0.3,pcm(3)-0.3],'String',sprintf('%0.3g N horizontal force',step(n).forceVector(1)));
            end
            
        set(lineData(i).time,'Position',[pcm(1)-1+0.1,pcm(2)+0.1+0.4,pcm(3)+0.5],'String',sprintf('%0.3g s',step(n).t(j)));
        set(lineData(i).velocity,'Position',[pcm(1)-1+0.1,pcm(2)+0.3+0.4,pcm(3)+0.3],'String',sprintf('[%0.2g, %0.2g, %0.2g] m/s walking speed',stepVelocity));
        end;
             
        end
    
        drawnow
        
        % Variable drawrate for real time display.
        if ~(movie); if(mod(drawNum,2)==0); tic; % Measure everyother step.
            else; if(toc<dataStepTime*drawRate) % Adjust drawrate depending on animation time.
                    drawRate=drawRate-1;
                else; drawRate=drawRate+1; end; end; end
        j = j + drawRate; drawNum = drawNum + 1;
        
        % Make movie option
        if movie;
            set(mf, 'Position', [100, 100, 1120, 840]);
            mov(mn) = getframe(figNum); mn = mn + 1;
        end
end
end

% Make movie option
if movie
    vidObj = VideoWriter(movieName,'Uncompressed AVI');
    vidObj.FrameRate = 30; open(vidObj);
    for i=1:size(mov,2) % Make actual movies from individual frames.
        writeVideo(vidObj,mov(i)); end;
    close(vidObj); end
  
end



