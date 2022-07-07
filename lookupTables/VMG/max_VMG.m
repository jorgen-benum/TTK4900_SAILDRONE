vmg8 = load('VMG_surgeVelocity_8ms.mat')
ang8 = load('VMG_angle_8ms.mat')

v8 = vmg8.VMG_vel
v8angles = ang8.VMG_angel

list1 = cos(v8angles*pi/180)'.*v8(11,:)
[a8max,b8max] = max(list1)
[a8min,b8min] = min(list1)

vmg4 = load('VMG_surgeVelocity_4ms.mat');

v4 = vmg4.VMG_vel;

list2 = cos(v8angles*pi/180)'.*v4(11,:);
[a4max,b4max] = max(list2)
[a4min,b4min] = min(list2)

vmg12 = load('VMG_surgeVelocity_12ms.mat');

v12 = vmg12.VMG_vel;

list3 = cos(v8angles*pi/180)'.*v12(11,:);
[a4max,b4max] = max(list3)
[a4min,b4min] = min(list3)

vmg16 = load('VMG_surgeVelocity_16ms.mat');

v16 = vmg16.VMG_vel;

list4 = cos(v8angles*pi/180)'.*v16(11,:);
[a4max,b4max] = max(list4)
[a4min,b4min] = min(list4)

tacklist_plus = [44, 43, 43, 44;
                0.62, 1.67, 2.66, 3.29;
                0.86, 2.28, 3.62, 4.57];
        
tacklist_minus = [163 ,163, 164, 164;
                  0.69, 1.76, 2.9, 4.08;
                  0.72, 1.84, 3.02, 4.24];