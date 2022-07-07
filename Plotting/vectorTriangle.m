function vectorTriangle(plot_position, scaling, U, U_flow, colorU, colorU_flow, colorU_relative, nameU, nameU_flow, nameU_relative)
% create vector triangle for velocities 
Ux = [plot_position(2),plot_position(2)+scaling*U(2)];
Uy = [plot_position(1),plot_position(1)+scaling*U(1)];
V_w_x = [plot_position(2)+scaling*(U(2)-U_flow(2)),plot_position(2)+scaling*U(2)];
V_w_y = [plot_position(1)+scaling*(U(1)-U_flow(1)),plot_position(1)+scaling*U(1)];
U_r_x = [plot_position(2),plot_position(2)+scaling*(U(2)-U_flow(2))];
U_r_y = [plot_position(1),plot_position(1)+scaling*(U(1)-U_flow(1))];
if norm(U) ~= 0  
    a = annotation('textarrow',Ux,Uy,'String',nameU);
    a.Color = colorU;
end
if norm(U_flow) ~= 0 
    b = annotation('textarrow',V_w_x,V_w_y,'String',nameU_flow);
    b.Color = colorU_flow;
end
if (norm(U_flow) ~= 0) && norm(U) ~= 0
    c = annotation('textarrow',U_r_x,U_r_y,'String',nameU_relative);
    c.Color = colorU_relative;
end

end

