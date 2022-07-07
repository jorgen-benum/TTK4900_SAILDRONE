function pathSpeedPlotter(position, speed)

patch([position(2,:) nan],[position(1,:) nan],[speed' nan],[speed' nan], 'edgecolor', 'interp'); 
colorbar;colormap(jet);

c = colorbar;
c.Label.String = 'Velocity [m/s]';
c.FontSize = 12;

end

