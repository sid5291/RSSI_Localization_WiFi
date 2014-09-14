% AP3 10.0.0.44 001b.0cfc.17e0 - 1
% NoT 10.0.0.43 001b.d571.8ebc - 2
% Pur 10.0.0.42 0018.74c4.5200 - 3
% AP4 10.0.0.41 001b.d59f.60fc - 4

function[] = path_loss()
t=tcpip('0.0.0.0', 65401, 'NetworkRole', 'server','InputBufferSize',1000); % Location Server at 65401
% t2=tcpip('0.0.0.0', 65402, 'NetworkRole', 'server','InputBufferSize',1000);
z_xs = zeros(41,35);
h=[ 1 1 1; 0.3 0.1 0.9; 0.6 0.9 0.1;0.9 0.3 0.1;0.7 0.2 0.5;1 1 0.5;1 0 0];
position = zeros(40,34); % Position Matrix Used to Trak Movement
position(1,1) = 3; % Router Positions
position(40,34) = 2;
position(40,1) = 1;
position(1,34) = 4;
routers = 4;
filter_circ = [0 0 4]; 
filter_init = 0; 
room = [0 0 1 1; 1 1 0 0] % Walls between user and routers depending on Room No.
rssi = zeros(1,4);  
walls = zeros(1,4);
index = 1;
circle = zeros(3);
distance = zeros(1,4);
load('Path_loss_data.mat') % Load Calibration Data
load('Position_rtr.mat');  % Router Positions
    pathlosseq = fittype(@(a,Iw,x,y) (10.^(-1*((x)+(53)-Iw*y)/(10*a))),'independent',{'x','y'},'dependent','z') % Curve Fit For given data to Calculate a and Iw 
    Path_Loss_Model=fit([x,y],z,pathlosseq) % x - rssi value; y - wall number; z - distance;  Iw - WAF; a -alpha (path loss gradient);
fopen(t) % Listen For Connection to server from Mobile user
% fopen(t2); % Currently un tested NFC Code
% if(~isempty(nfc_data))
%     nfc_data = str2num(nfc_data)
%     filter_circ = [nfc_data(1) nfc_data(2) 4];
%     data = str2num(server_input(t,1));
%     x=[x;((data(3:3:12).'))];
%     for i = 1:4
%         y=[y;(room(data(1)).')];
%         dist = sqrt((AP_Position(i,1)- nfc_data(1)).^2) - ((AP_Position(i,2) - nfc_data(1)).^2);
%         z =[z;dist];
%     end    
%     pathlosseq = fittype(@(a,Iw,x,y) (10.^(-1*((x)+53-Iw*y)/(10*a))),'independent',{'x','y'},'dependent','z')
%     Path_Loss_Model=fit([x,y],z,pathlosseq) % x - rssi value; y - wall number; z - distance;  Iw - WAF; a -alpha (path loss gradient);
% end
while(1)
rssi = zeros(1,4); 
% Currently untested NFC Code
% nfc_data = server_input(t2,2); 
% if(~isempty(nfc_data))
%     nfc_data = str2num(nfc_data)
%     data = str2num(server_input(t,1));
%     filter_circ = [nfc_data(1) nfc_data(2) 4];
%     x=[x;((data(3:3:12).'))];
%     for i = 1:4
%         y=[y;(room(data(1)).')];
%         dist = sqrt((AP_Position(i,1)- nfc_data(1)).^2) - ((AP_Position(i,2) - nfc_data(1)).^2);
%         z =[z;dist];
%     end    
%     pathlosseq = fittype(@(a,Iw,x,y) (10.^(-1*((x-30)+53-Iw*y)/(10*a))),'independent',{'x','y'},'dependent','z')
%     Path_Loss_Model=fit([x,y],z,pathlosseq) % x - rssi value; y - wall number; z - distance;  Iw - WAF; a -alpha (path loss gradient);
% end
for i=1:10
   data = str2num(server_input(t,1)); % Get 10 consecutive RSSI Data Points from Mobile user
   data_write(index,:) = data;
   rssi = rssi + (data(3:3:12)-30);
   room_index = data(1);
end
rssi = rssi./10 % Average out the RSSI values 
for i = 1:routers
    wall(i) = room(room_index,i);
    distance(i) = Path_Loss_Model(rssi(i),walls(i)); % Calculate Distance from eah router using path loss model
end
for i = 1:3   % Sort for 3 least distances
    [val,index] = min(distance);
    distance(index) = NaN;
    circle(i,:) = [AP_Position(index,:),floor(val*100)];   
end
  pts12 = intersectCircles(circle(1,:),circle(2,:)) % check for intersecting points between Circle 1 and 2
    if(isnan(pts12) == 1)
        continue;
    end
  pts32 = intersectCircles(circle(3,:),circle(2,:)) % check for intersecting points between Circle 3 and 2
  if(isnan(pts32) == 1)
        continue;
  end
  pts13 = intersectCircles(circle(1,:),circle(3,:)) % check for intersecting points between Circle 1 and 3
  if(isnan(pts13) == 1)
        continue;
  end
  % choose the 3 points that form the region of intersection for the three circles
  if(iscircle(circle(3,:),pts12(1,1),pts12(1,2)) < 0)
      final_point(1,:)= pts12(1,:);
  else
      final_point(1,:)= pts12(2,:);
  end
   if(iscircle(circle(2,:),pts13(1,1),pts13(1,2)) < 0)
      final_point(2,:)= pts13(1,:);
   else
      final_point(2,:)= pts13(2,:);
   end
    if(iscircle(circle(1,:),pts32(1,1),pts32(1,2)) < 0)
      final_point(3,:)= pts32(1,:);
    else
      final_point(3,:)= pts32(2,:);
    end
    Final_Coordinates = floor(centroid(final_point)) % Find Centroid of the region intersection
    if(min(Final_Coordinates) <= 0)
        continue;
    end
    if((Final_Coordinates(1) > 400) || (Final_Coordinates(2) > 340) )
        continue;
    end
     new_coords = [ceil(Final_Coordinates(1)/10),ceil(Final_Coordinates(2)/10)]
    if(filter_init == 0) % Intialize filter
        filter_circ = [new_coords(1) new_coords(2) 10]
        filter_init =1;
    end  
    % Store filter Region and check if calculated point lies within it
    if(iscircle(filter_circ,new_coords(1),new_coords(2))< 0)
        position(new_coords(1),new_coords(2)) = 5;
       filter_circ = [new_coords(1) new_coords(2) 10];
        figure(1)
        surf(z_xs,position)
        axis ij
        axis([1 35 1 41 0 1])
        colormap(h)
        colorbar
    end
 end
fclose(t);
fclose(t2)
end

function [result] = iscircle(circ,cor_x,cor_y)
    val=sqrt(((cor_x-circ(1))^2)+((cor_y-circ(2))^2))-circ(3);
    result=sign(val);
end



