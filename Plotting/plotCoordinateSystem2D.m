function plotCoordinateSystem2D(coordinatesystem)

      x = coordinatesystem(:,1:2);
      y = coordinatesystem(:,3:4);
      z = coordinatesystem(:,5:6);
%     plot(x(2,:), x(1,:), 'r', 'LineWi')
%     plot(y(2,:), y(1,:), 'g')
      delta_x = x(:,2) - x(:,1);
      delta_y = y(:,2) - y(:,1);

      quiver(x(2,1),x(1,1),delta_x(2),delta_x(1),'r','LineWidth',2)
      quiver(y(2,1),y(1,1),delta_y(2),delta_y(1),'g','LineWidth',2) 


end

