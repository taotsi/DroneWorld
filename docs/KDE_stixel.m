%function [] = KDE_stixel()
% pict 记录运行此处，多帧运行
clearvars -EXCEPT pict
close all;clc
% 记录障碍物列提取的结果，（x,y）为二维映射坐标，h为障碍物下端的世界坐标高度，z为上端世界坐标高度
pict =3; 
Stixel_result_x = [];
Stixel_result_y = [];
% 坐标位置(x,y)
Stixel_result_z = [];
Stixel_result_h = [];

Stixel_result_vb = [];
Stixel_result_vt = [];
%记录该stixel是否超出视野，超出记为1 ，下面两个变量分别记录上方和下方超出视野
Beyond_view_flag = [];
below_view_flag = [];
% 记录障碍物提取个数
Stixel_number = 0;

% 读取并转换深度数据（数据原格式为物体像平面的距离，单位m）
%pict = 1;
RGBshow = imread('ZED采集/7/1.png');
Filename = strcat('ZED采集/7/',num2str(pict), '.pfm');
Picture_num = pict + 1;
pict = pict + 1;
fid = fopen(Filename);
fscanf(fid,'%c',[1,3]);
cols = fscanf(fid,'%f',1);
rows = fscanf(fid,'%f',1);
fscanf(fid,'%f',1);
fscanf(fid,'%c',1);
DEPTH = fread(fid,[cols,rows],'single');
DEPTH(DEPTH == Inf) = 0;
DEPTH = rot90(DEPTH);
D = zeros(rows,cols);
for v = 1:cols
    for u = 1:rows
        if (DEPTH(u,v) > 1)
            D(u,v) = 448.1328 * 0.12  / (DEPTH(rows-u+1,v))  + 0*rand(1);
        else
            D(u,v) = 0;    
        end
    end
end
%w2=fspecial('average',[1 20]);
%D=imfilter(D,w2,'replicate');
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%读取txt中存储的无人机状态数据

uav_x = 0;
uav_y = 0;
uav_z = 0;
UAV_z = ones(200,1);
UAV_roll = zeros(200,1);
UAV_pitch = zeros(200,1);
UAV_yaw = zeros(200,1);

% 计算像素坐标向世界坐标变换的变量,此处的角度需要确定正负号
CosValue = cos(UAV_pitch(Picture_num));
SinValue = -sin(UAV_pitch(Picture_num));
figure(1);
% 旋转深度图像，校正roll角度，由于角度参数不准确，此处先不做修正....
histeqSample = histeq(D);
imshow(histeqSample);


[rows,cols] = size(D);
%for n = 200:1:201
%绘图选择列坐标
w=100;  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(3);
plot(D(1:rows,w));
%axis([0 375 0 260])
%绘制某一列深度视差
A_plot = D(1:rows,w);
%A = 10*A;
% 设置stixel宽度
stixel_width = 7;
% 设置直方图的统计分辨率
width=64;


%设置相机参数
fx = 448.1328;
fy = 448.1328;
v0 = rows/2;
u0 = cols/2;
Baseline = 0.12;


% 由于是仿真数据，需要自己限定范围
% 根据相机参数和视差计算最小分辨率和最大探测距离范围
Z_min = 0.5;
Z_max = 30;
Disparity_min = fx * Baseline / Z_max;
Disparity_max = fx * Baseline / Z_min;
% 设置物体有效高度，最小物体的高度
detal_y = 0.3;
sitxel_cnt = 1;
% 设置核密度估计的宽度 此处如果过于小，容易引入量化误差，即相邻stixel深度轻微变化无法表达，出现阶梯
Kernel_width = 1000;

figure(4);
[x,N] = hist(A_plot,width);
bar(x);
    
%核密度估计

W5_gauss = [0.00170354, 0.109878, 0.440655, 0.109878, 0.0017035];
W7_gauss = [0.00038771, 0.01330373, 0.11098164, 0.22508352, 0.11098164, 0.01330373, 0.00038771];
W9_guass = [0.0561    0.1979    0.4868    0.8354    1.0001    0.8354    0.4868    0.1979    0.0561];
W17_guass = [0.0561    0.1103    0.1979    0.3247    0.4868    0.6670    0.8354    0.9561    1.0001    0.9561 0.8354    0.6670    0.4868    0.3247    0.1979    0.1103    0.0561];
W5_trian = [0.2 0.6 1 0.6 0.2];
W9_trian = [0.05 0.1 0.2 0.4 0.6 0.8 0.6 0.4 0.2 0.1 0.05];
W13_trian = [0.1 0.3 0.5 0.7 0.9 1.1 1.3 1.1 0.9 0.7 0.5 0.3 0.1];
W19_trian = [0.1 0.2 0.3 0.4 0.5 0.6 .07 0.8 0.9 1 0.9 0.8 0.7 0.6 0.5 0.4 0.3 0.2 0.1];
W25_trian = [0.1 0.2 0.3 0.4 0.5 0.6 .07 0.8 0.9 1 1.1 1.2 1.3 1.4 1.3 1.2 1.1 1 0.9 0.8 0.7 0.6 0.5 0.4 0.3 0.2 0.1];

% 核密度估计的分辨率
[f,xi]=ksdensity(A_plot);
figure(5);
plot(f);
%%
    % 绘制调试部分
    clear A;
    A = [];
    KDE_D = zeros(Kernel_width,1);
    u = w;
    for v =1:rows
        if ((D(v,u)>Disparity_min)&&(D(v,u)<Disparity_max))
            A = [A D(v,u)];
            kde_cnt = round(D(v,u) * Kernel_width / Disparity_max);
            for k = -9:9
                if ((k + kde_cnt) > 1)
                    % 此处 -1 的主要原因是 
                    KDE_D(k + kde_cnt) = KDE_D(k + kde_cnt) + W19_trian(k+10);
                end
            end
        end
    end
    figure(6);
    plot(KDE_D);
    
    % 若噪声太大可以采用IIR滤波器过滤毛刺噪声
    B = zeros(3,1);
    A = zeros(3,1);
    B(1)=1;B(2)=2;B(3)=1;
	A(1)=1;A(2)=-0.837000;A(3)=0.42398;
	Gain=0.146747;
    w_x = zeros(3,1);
    w_y = zeros(3,1);
    KDE_filter = zeros(Kernel_width,1);
	for i = 1:1:Kernel_width
		w_x(1)=KDE_D(i);
		w_y(1)=(B(1)*w_x(1)+B(2)*w_x(2)+B(3)*w_x(3))*Gain-w_y(2)*A(2)-w_y(3)*A(3);
		KDE_filter(i)=w_y(1)/A(1);
		w_x(3)=w_x(2);w_x(2)=w_x(1);
		w_y(3)=w_y(2);w_y(2)=w_y(1);
    end
 
    
    
% Filename1 = strcat('Image/',num2str(pict-5), '.png');

 figure(2);hold on;
 imshow(RGBshow);
%figure(8);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 提取障碍物列
for u = 3:stixel_width:cols
    clear A;
    A = [];
    % 产生响应和密度宽度的存储
    KDE_D = zeros(Kernel_width,1);
    % 滤除视差较小，距离大于100米的信息
    % 采用概率密度估计统计分布
    for v =1:rows
        if ((D(v,u)>Disparity_min)&&(D(v,u)<Disparity_max))
            A = [A D(v,u)];
            kde_cnt = round(D(v,u) * Kernel_width / Disparity_max);
            for k = -9:9
                if ((k + kde_cnt) > 1)
                    % 此处 -1 的主要原因是 
                    KDE_D(k + kde_cnt) = KDE_D(k + kde_cnt) + W19_trian(k+10);
                end
            end
        end
    end
    
    % 找出物体，即大于一定阈值50cm高度的物体
    % 通过核密度估计函数找出物体的深度均值和方差
    Object_cnt = 0;
    Object_mean = zeros(6,1);
    Object_depth_value = zeros(6,1);
    Object_detla = zeros(6,1);
    t = 3;
    while (t < Kernel_width-2 )
        % 需要计算确切的结果
        %delt_y = (v1-v2)*baseline/disparity;
        if ((KDE_D(t)*Baseline) > (0.5 * t * Disparity_max / Kernel_width))
            if ((KDE_D(t)>KDE_D(t-1))&&(KDE_D(t)>KDE_D(t+1)))
                if ((KDE_D(t)>KDE_D(t-2))&&(KDE_D(t)>KDE_D(t+2)))
                    if ((KDE_D(t)>KDE_D(t-3))&&(KDE_D(t)>KDE_D(t+3)))
                        Object_cnt = Object_cnt + 1;
                        Object_mean(Object_cnt) = t;
                        % 离散后的数据转换为真实值
                        Object_depth_value(Object_cnt) = t * Disparity_max / Kernel_width;
                    end
                end
            end
        end
        t = t + 1;
    end
    % 检测物体的数量Object_cnt个
    for o=1:Object_cnt
        detla_cnt = 1;
        while(( 0.7 * KDE_D(Object_mean(o))  <  KDE_D( Object_mean(o)-detla_cnt )) || ( 0.7 * KDE_D(Object_mean(o))  <  KDE_D( Object_mean(o) + detla_cnt)) )
            detla_cnt = detla_cnt + 1;
        end
        Object_detla(Object_cnt) = 2.0 * detla_cnt * Disparity_max / Kernel_width;
    end
    
    % 计算detal_y(30cm)的高度在对应视差上的像素高度，作为物体筛选的最小值。
    %detal_v = detal_y * Dis_maxnumber1 /Baseline;
    
    v=1;
    % 选择变量，进行物体判断或滤波
    choose_case = 1;
    % 判断是否有物体
    judge_object = 1;
    % 计算滤波数量
    filter_object = 2;
    % 当前物体的深度均值和方差
    Disparity_temp = 0;
    Detla_temp = 0;
    % 记录物体的像素数量，做阈值等判定
    judge_cnt = 0;
    
    clear A;
    A = D(1:rows,u);
    while (v<=rows) 
        
        switch choose_case
            %选择判断是否有物体存在
            case judge_object
                for o = 1:Object_cnt
                    if (abs(A(v)-Object_depth_value(o)) < Object_detla(o))
                        % 当物体深度或视差与某一物体视差或深度在一定范围内时，跳入滤波模式
                        Disparity_temp = Object_depth_value(o);
                        Detla_temp = Object_detla(o);
                        choose_case = filter_object;
                        v = v - 1;
                    end
                end
                v = v + 1;
            %计算属于该物体的像素个数
            case filter_object
                if (abs(A(v)-Disparity_temp) < Detla_temp)
                    judge_cnt =judge_cnt + 1;
                    v = v + 1;
                    if (v>=(rows))
                        Stixel_result_vb = [Stixel_result_vb (v-judge_cnt)];
                        Stixel_result_vt = [Stixel_result_vt (v)];
                        
                        %rectangle('position',[u,rows-v,stixel_width,judge_cnt],'facecolor','g');
                        sum_temp = 0;
                        for l=(v-judge_cnt+2):v-4
                            sum_temp = sum_temp + D(l,u);
                        end
                        Disparity_temp = sum_temp / (judge_cnt-7);
                        
                        x_temp = (u-u0) * Baseline / Disparity_temp;
                        % 计算的高度数据为的h轴3下方
                        h_temp = -(fx * SinValue + (v-judge_cnt-v0) * CosValue) * Baseline / Disparity_temp + UAV_z(Picture_num);
                        % 障碍物低端的纵坐标
                        y_temp = -(fx * SinValue + (v-v0) * CosValue) * Baseline / Disparity_temp + UAV_z(Picture_num);
                        z_temp = (fx * CosValue - (v-judge_cnt-v0) * SinValue) * Baseline / Disparity_temp;
                        % 计算相机坐标位置，转换到世界坐标系下
                        Position_camera = [x_temp z_temp h_temp 1];
                        Position_world = Position_camera;
                        x_world = Position_world(1);
                        y_world = Position_world(2);
                        z_world = Position_world(3);
                        if (z_world<1)
                            z_world = 0;
                        end
                        if (y_temp<1)
                            y_temp = 0;
                        end
                        if (abs(x_world)<10)
                            if ((v-judge_cnt)<3)  
                                Beyond_view_flag = [Beyond_view_flag 1];
                            else
                                Beyond_view_flag = [Beyond_view_flag 0];
                            end      
                            if (v>=rows-2)  
                                below_view_flag = [below_view_flag 1];
                            else
                                below_view_flag = [below_view_flag 0];
                            end
                            if ((h_temp>0)&&((z_world-y_temp)>0.5))
                                Stixel_result_x = [Stixel_result_x x_world];
                                Stixel_result_y = [Stixel_result_y y_world];
                                Stixel_result_z = [Stixel_result_z z_world];
                                Stixel_result_h = [Stixel_result_h y_temp];
                                Stixel_number = Stixel_number + 1;
                                line([u,u],[v,v-judge_cnt],'linestyle','-', 'LineWidth',2);
                            end                           
                        end
                        v = v + 1;
                        judge_cnt = 0;
                    end
                elseif (judge_cnt>10)
                    tempcnt = 0;
                    h_filter = ceil(1 * Disparity_temp / Baseline);

                    for h=v:min(v+h_filter,rows)
                        if (abs(A(h)-Disparity_temp) < Detla_temp)
                            tempcnt = tempcnt + 1;
                        end
                    end
                    if (tempcnt <= h_filter/2)
                        
                        Stixel_result_vb = [Stixel_result_vb (v-judge_cnt)];
                        Stixel_result_vt = [Stixel_result_vt (v)];
                        
                        sum_temp = 0;
                        for l=(v-judge_cnt+2):v-4
                            sum_temp = sum_temp + D(l,u);
                        end
                        Disparity_temp = sum_temp / (judge_cnt-7);
                        
                        %rectangle('position',[u,rows-v,stixel_width,judge_cnt],'facecolor','g');
                        x_temp = (u-u0) * Baseline / Disparity_temp;
                        % 计算的高度数据为的h轴3下方  y_temp 是列像素低端  h_temp是列像素障碍物顶端
                        h_temp = -(fx * SinValue + (v-judge_cnt-v0) * CosValue) * Baseline / Disparity_temp + UAV_z(Picture_num);
                        % 障碍物低端的纵坐标
                        y_temp = -(fx * SinValue + (v-v0) * CosValue) * Baseline / Disparity_temp + UAV_z(Picture_num);
                        z_temp = (fx * CosValue - (v-judge_cnt-v0) * SinValue) * Baseline / Disparity_temp;
                        Position_camera = [x_temp z_temp h_temp 1];
                        Position_world = Position_camera;
                        x_world = Position_world(1);
                        y_world = Position_world(2);
                        z_world = Position_world(3);

                        if (z_world<1)
                            z_world = 0;
                        end
                        if (y_temp<1)
                            y_temp = 0;
                        end
                        if (abs(x_world)<10)
                             if ((h_temp>0)&&(abs(z_world-y_temp)>0.5))
                                if ((v-judge_cnt)<3)  
                                    Beyond_view_flag = [Beyond_view_flag 1];
                                else
                                    Beyond_view_flag = [Beyond_view_flag 0];
                                end
                                if (v>=rows-2)  
                                    below_view_flag = [below_view_flag 1];
                                else
                                    below_view_flag = [below_view_flag 0];
                                end
                                
                                Stixel_result_x = [Stixel_result_x x_world];
                                Stixel_result_y = [Stixel_result_y y_world];
                                Stixel_result_z = [Stixel_result_z z_world];
                                Stixel_result_h = [Stixel_result_h y_temp];
                                Stixel_number = Stixel_number + 1;
                                line([u,u],[v,v-judge_cnt],'linestyle','-', 'LineWidth',2);
                            end                           
                        end
                        judge_cnt = 0;
                        choose_case = judge_object;
                    else 
                        v = min(v+h_filter,rows);
                        judge_cnt = min(judge_cnt+h_filter,rows);
                    end
                    
                else
                    choose_case = judge_object;
                    judge_cnt = 0;                  
                end               
        end      
    end
    if (u>1000)
        
    end
end

figure(11);
% 绘制
for s = 1:Stixel_number
    plot3([Stixel_result_x(s) Stixel_result_x(s)],[Stixel_result_y(s) Stixel_result_y(s)], ...
        [Stixel_result_h(s) Stixel_result_z(s)], 'LineWidth',2);
    hold on;
    grid on
end


%& 聚类分三个步骤，首先在聚集上进行聚类，不管高度范围，仅考虑x,y方向，然后将聚类一类的看做一个墙体，在高度范围上寻找窗口
%  窗口可以通过无人机的话进行聚类拆分，即最终的到的结果是同一类内部高度相同。
%  采用的主要方法是用cluster_flag 标记该stixel属于哪儿一类
%% 距离聚类
cluster_flag = [1];
cluster_number = 1;
plane_number = 0;
% 记录聚类中某一类别的序号
result_cluster_tempx = [Stixel_result_x(1)];
result_cluster_tempy = [Stixel_result_y(1)];
result_cluster_temph = [Stixel_result_h(1)];
result_cluster_tempz = [Stixel_result_z(1)];
% 此处聚类算法可以考虑通过每列深度像素获取的物体个数进行聚类。

for p = 2:Stixel_number
    for c = 1:1:cluster_number
        % 记录聚类对比点，即从左向右不断搜索聚类距离，以当前最靠右方的作为聚类中心
        cluster_center_x = result_cluster_tempx(c);
        cluster_center_y = result_cluster_tempy(c);
        cluster_center_h = result_cluster_temph(c);
        cluster_center_z = result_cluster_tempz(c);
        distance_xy = sqrt(((Stixel_result_x(p)-cluster_center_x)^2+(Stixel_result_y(p)-cluster_center_y)^2 ));
        distance_z = 0;%abs(Stixel_result_h(p) - cluster_center_h); % z坐标的差异，即低端的距离
        distance_h = 0;%abs(Stixel_result_z(p) - cluster_center_z); % 高度的差异
        if ((distance_xy<2)&&(distance_z<1)&&(distance_h<1))
            cluster_flag = [cluster_flag c];
            result_cluster_tempx(cluster_number) = Stixel_result_x(p);
            result_cluster_tempy(cluster_number) = Stixel_result_y(p);
            result_cluster_temph(cluster_number) = Stixel_result_h(p);
            result_cluster_tempz(cluster_number) = Stixel_result_z(p);
            break;
        elseif (c == cluster_number)
            % 当聚类出现大于聚类中心的，则加入新的聚类，聚类数量加一
            cluster_number = cluster_number + 1;
            cluster_flag = [cluster_flag cluster_number];
            result_cluster_tempx = [result_cluster_tempx Stixel_result_x(p)];
            result_cluster_tempy = [result_cluster_tempy Stixel_result_y(p)];
            result_cluster_temph = [result_cluster_temph Stixel_result_h(p)];
            result_cluster_tempz = [result_cluster_tempz Stixel_result_z(p)];
            break;
        end
    end
end

% 设置无人机飞行的安全高度和宽度，从而进行高度范围内的滤波处理
Height_safe = 1;
Width_safe  = 2;

%（x,y）为二维映射坐标，h为障碍物下端的世界坐标高度，z为上端世界坐标高度
%% 高度滤波
% 核心思想是聚类内部以高度的最大值和最小值作为该聚类“墙体”的高度最大值和最小值，对比其他pillar的高度，计算补集
% 补集的结果与周围进行相邻求交集，若交集大于无人机可通行高度，则继续向两边搜索，若无法通过则认为该pillar是堵死的
% 用Z_max,Z_min表达其高度。
% 为简化搜索次数，采用二分法搜索，即首先查找最中间的一个pillar，若大于则直接拆分，若小于无人机高度则堵死，拆分为两个待搜索块，继续针对两个待搜索块查找中间pillar是否存在无人机可穿行的高度

for f = 1:cluster_number
    Bef_filter_xx = [];
    Bef_filter_yy = [];
    Bef_filter_zz = [];
    Bef_filter_hh = [];
    
    % 将分到某一类的stixel放在xx，yy中，即二维平面上
    serial_number_stixel = [];
    for s = 1:Stixel_number
        if(cluster_flag(s) == f)
            % 将同一聚类的stixel提取出来，存在Bef_filter中，即滤波前存储
            Bef_filter_xx = [Bef_filter_xx Stixel_result_x(s)];
            Bef_filter_yy = [Bef_filter_yy Stixel_result_y(s)];
            Bef_filter_hh = [Bef_filter_hh Stixel_result_z(s)];    % 转变后hh表达高度，即上方坐标
            Bef_filter_zz = [Bef_filter_zz Stixel_result_h(s)];    % 转变后zz表达底部坐标
            serial_number_stixel = [serial_number_stixel s];
        end
    end
    % 获取聚类内部的高度最大值和最小值，用于过滤。
    Zmax_temp = max(Bef_filter_hh);
    Zmin_temp = min(Bef_filter_zz);
    % 求每个聚类内部stixel的高度补集
    BefCluster_number = size(Bef_filter_xx);
    % 分别记录补集的上部分和下部分
    Complement_Bef_z1 = zeros(BefCluster_number(2),2);
    Complement_Bef_z2 = zeros(BefCluster_number(2),2);
    % 填补由于视野有限导致近处看到的stixel偏低
    max_beyongd_temp = 0;
    if(~isempty(Beyond_view_flag))
        for s = 1:BefCluster_number(2)
            if (Beyond_view_flag(s) == 1)
                if (Bef_filter_hh(s) > max_beyongd_temp)
                    max_beyongd_temp = Bef_filter_hh(s);
                end
            end
        end
        % 对高度超过视野的进行推理填补
        for s = 1:Stixel_number       
            if(cluster_flag(s) == f)
                if (Beyond_view_flag(s) == 1)
                    Stixel_result_z(s) = max_beyongd_temp;    
                end
            end
        end
        
        % 对正在处理的数据进行高度视野有限产生误差进行填补
        for s = 1:BefCluster_number(2)
            if (Beyond_view_flag(s) == 1)
                Bef_filter_hh(s) = max_beyongd_temp;
            end
        end
    end
    
    % 填补由于视野有限导致近处看到的stixel偏高
    min_beyongd_temp = Inf;
    if(~isempty(below_view_flag))
        for s = 1:BefCluster_number(2)
            if (below_view_flag(s) == 1)
                if (Bef_filter_hh(s) < min_beyongd_temp)
                    min_beyongd_temp = Bef_filter_hh(s);
                end
            end
        end
    % 对高度超过视野的进行推理填补
    
        for s = 1:Stixel_number
            if(cluster_flag(s) == f)
                if (below_view_flag(s) == 1)
                    Stixel_result_z(s) = min_beyongd_temp;    
                end
            end
        end
        
        % 对正在处理的数据进行高度视野有限产生误差进行填补
        for s = 1:BefCluster_number(2)
            if (below_view_flag(s) == 1)
                Bef_filter_zz(s) = min_beyongd_temp;
            end
        end
    end
   
    % 对正在处理的数据进行高度视野有限产生误差进行填补
    for s = 1:BefCluster_number(2)
        if (Beyond_view_flag(s) == 1)
            Bef_filter_hh(s) = max_beyongd_temp;
        end
    end
    % 对填补视野有限导致的偏差，求解补集。
    for Bef_cnt = 1:BefCluster_number(2)
        if ((Zmax_temp-Bef_filter_hh(Bef_cnt))>Height_safe)
            Complement_Bef_z1(Bef_cnt,1) = Zmax_temp;
            Complement_Bef_z1(Bef_cnt,2) = Bef_filter_hh(Bef_cnt);          
        end
        if ((Bef_filter_zz(Bef_cnt)-Zmin_temp)>Height_safe)
            Complement_Bef_z2(Bef_cnt,1) = Bef_filter_zz(Bef_cnt);
            Complement_Bef_z2(Bef_cnt,2) = Zmin_temp;
        end
    end
    
    % 前面是对高度超出视野的进行填补，填补后进一步计算补集与相邻的交集
    i = 1;
    while ( i < BefCluster_number(2)-1 ) 
        test_zmax = Complement_Bef_z1(i,1);
        test_zmin = Complement_Bef_z1(i,2);
        if ((test_zmax-test_zmin)>Height_safe)
            for j = i+1:BefCluster_number(2)
                % 将待检测的列高度赋值
                Compared_zmax1 = Complement_Bef_z1(j,1);
                Compared_zmin1 = Complement_Bef_z1(j,2);
                Compared_zmax2 = Complement_Bef_z2(j,1);
                Compared_zmin2 = Complement_Bef_z2(j,2);
                if ((test_zmax>Compared_zmin1)&&(test_zmin<Compared_zmax1))
                    test_zmax = min(test_zmax,Compared_zmax1);
                    test_zmin = max(test_zmin,Compared_zmin1);
                elseif((test_zmax>Compared_zmin2)&&(test_zmin<Compared_zmax2))
                    test_zmax = min(test_zmax,Compared_zmax2);
                    test_zmin = max(test_zmin,Compared_zmin2);
                else
                    distance_i_j = sqrt((Bef_filter_xx(i)-Bef_filter_xx(j-1))^2 + (Bef_filter_yy(i)-Bef_filter_yy(j-1))^2);
                    if (distance_i_j > Width_safe)
                        % 若中间出现大于无人机可安全通行的区域时，将同一聚类进行拆分。
                        % 将后续的stixel进行拆分，得到新的分组
                        % 记录上方和下方是否拆分开
                        clustering_add_cnt1 = 0;
                        clustering_add_cnt2 = 0;
                        for w = i:j
                            if (((Complement_Bef_z1(w,1)>=test_zmax)&&(Complement_Bef_z1(w,2)<=test_zmin))||  ...
                                ((Complement_Bef_z2(w,1)>=test_zmax)&&(Complement_Bef_z2(w,2)<=test_zmin)))
                                cluster_flag(serial_number_stixel(w)) = cluster_number + 1;
                                clustering_add_cnt1 = 1;
                            end
                        end
                        %总的聚类发生改变
                        cluster_number = cluster_number + clustering_add_cnt1 + clustering_add_cnt2;
                        for w = j:BefCluster_number(2)
                            cluster_flag(serial_number_stixel(w)) = cluster_number + 1;
                        end
                        cluster_number = cluster_number + 1;
                        i = j;
                    end
                    break;
                end
                if (abs(test_zmax-test_zmin)<Height_safe)
                    distance_i_j = sqrt((Bef_filter_xx(i)-Bef_filter_xx(j-1))^2 + (Bef_filter_yy(i)-Bef_filter_yy(j-1))^2);
                    if (distance_i_j > Width_safe)
                        % 若中间出现大于无人机可安全通行的区域时，将同一聚类进行拆分。
                        % 将后续的stixel进行拆分，得到新的分组
                        % 记录上方和下方是否拆分开
                        clustering_add_cnt1 = 0;
                        clustering_add_cnt2 = 0;
                        for w = i:j
                            if (((Complement_Bef_z1(w,1)>=test_zmax)&&(Complement_Bef_z1(w,2)<=test_zmin))||  ...
                                ((Complement_Bef_z2(w,1)>=test_zmax)&&(Complement_Bef_z2(w,2)<=test_zmin)))
                                cluster_flag(serial_number_stixel(w)) = cluster_number + 1;
                                clustering_add_cnt1 = 1;
                            end
                        end
                        %总的聚类发生改变
                        cluster_number = cluster_number + clustering_add_cnt1 + clustering_add_cnt2;
                        for w = j:BefCluster_number(2)
                            cluster_flag(serial_number_stixel(w)) = cluster_number + 1;
                        end
                        cluster_number = cluster_number + 1;
                        i = j;
                    end
                    break;
                end
            end
        end
        
        test_zmax = Complement_Bef_z2(i,1);
        test_zmin = Complement_Bef_z2(i,2);
        if ((test_zmax-test_zmin)>Height_safe)
            for j = i+1:BefCluster_number(2)
                % 将待检测的列高度赋值
                Compared_zmax1 = Complement_Bef_z1(j,1);
                Compared_zmin1 = Complement_Bef_z1(j,2);
                Compared_zmax2 = Complement_Bef_z2(j,1);
                Compared_zmin2 = Complement_Bef_z2(j,2);
                if ((test_zmax>Compared_zmin1)&&(test_zmin<Compared_zmax1))
                    test_zmax = min(test_zmax,Compared_zmax1);
                    test_zmin = max(test_zmin,Compared_zmin1);
                elseif((test_zmax>Compared_zmin2)&&(test_zmin<Compared_zmax2))
                    test_zmax = min(test_zmax,Compared_zmax2);
                    test_zmin = max(test_zmin,Compared_zmin2);
                else
                    distance_i_j = sqrt((Bef_filter_xx(i)-Bef_filter_xx(j-1))^2 + (Bef_filter_yy(i)-Bef_filter_yy(j-1))^2);
                    if (distance_i_j > Width_safe)
                        % 若中间出现大于无人机可安全通行的区域时，将同一聚类进行拆分。
                        % 将后续的stixel进行拆分，得到新的分组
                        % 记录上方和下方是否拆分开
                        clustering_add_cnt1 = 0;
                        clustering_add_cnt2 = 0;
                        for w = i:j
                            if (((Complement_Bef_z1(w,1)>=test_zmax)&&(Complement_Bef_z1(w,2)<=test_zmin))||  ...
                                ((Complement_Bef_z2(w,1)>=test_zmax)&&(Complement_Bef_z2(w,2)<=test_zmin)))
                                cluster_flag(serial_number_stixel(w)) = cluster_number + 1;
                                clustering_add_cnt1 = 1;
                            end
                        end
                        %总的聚类发生改变
                        cluster_number = cluster_number + clustering_add_cnt1 + clustering_add_cnt2;
                        for w = j:BefCluster_number(2)
                            cluster_flag(serial_number_stixel(w)) = cluster_number + 1;
                        end
                        cluster_number = cluster_number + 1;
                        i = j;
                    end
                    break;
                end
                if (abs(test_zmax-test_zmin)<Height_safe)
                    distance_i_j = sqrt((Bef_filter_xx(i)-Bef_filter_xx(j-1))^2 + (Bef_filter_yy(i)-Bef_filter_yy(j-1))^2);
                    if (distance_i_j > Width_safe)
                        % 若中间出现大于无人机可安全通行的区域时，将同一聚类进行拆分。
                        % 将后续的stixel进行拆分，得到新的分组
                        % 记录上方和下方是否拆分开
                        clustering_add_cnt1 = 0;
                        clustering_add_cnt2 = 0;
                        for w = i:j
                            if (((Complement_Bef_z1(w,1)>=test_zmax)&&(Complement_Bef_z1(w,2)<=test_zmin))||  ...
                                ((Complement_Bef_z2(w,1)>=test_zmax)&&(Complement_Bef_z2(w,2)<=test_zmin)))
                                cluster_flag(serial_number_stixel(w)) = cluster_number + 1;
                                clustering_add_cnt1 = 1;
                            end
                        end
                        %总的聚类发生改变
                        cluster_number = cluster_number + clustering_add_cnt1 + clustering_add_cnt2;
                        for w = j:BefCluster_number(2)
                            cluster_flag(serial_number_stixel(w)) = cluster_number + 1;
                        end
                        cluster_number = cluster_number + 1;
                        i = j;
                    end
                    break;
                end
            end
        end
    i = i + 1;
    end
end


%% 高度滤波 ， 将同一类别的的高度进行统一
for f = 1:cluster_number
    serial_number_stixel = [];
    filter_hh = [];
    filter_zz = [];
    for s = 1:Stixel_number
        if(cluster_flag(s) == f)
            filter_hh = [filter_hh Stixel_result_z(s)];    % 转变后hh表达高度，即上方坐标
            filter_zz = [filter_zz Stixel_result_h(s)];    % 转变后zz表达底部坐标
            serial_number_stixel = [serial_number_stixel s];
        end
    end
    max__temp = 0;
    min__temp = Inf;
    size_filter = size(filter_hh);
    for s = 1:size_filter(2)
        if (filter_hh(s) > max__temp)
            max__temp = filter_hh(s);
        end
        if (filter_zz(s) < min__temp)
            min__temp = filter_zz(s);
        end
    end
    % 对高度超过视野的进行推理填补
    for s = 1:Stixel_number
        if(cluster_flag(s) == f)
            Stixel_result_z(s) = max__temp;    
            Stixel_result_h(s) = min__temp;  
        end
    end

end

%% 平面提取
plane_k = zeros(cluster_number,1);
plane_b = zeros(cluster_number,1);
plane_x_min = [];
plane_x_max = [];
plane_y_min = [];
plane_y_max = [];
plane_z_min = [];
plane_z_max = [];
plane_number = 0;
last_p = [];
p = zeros(2,1);
figure(13);
line_k = [];
line_b = [];
for f = 1:cluster_number
    xx = [];
    yy = [];
    zz = [];
    hh = [];
    xx_temp = [];
    yy_temp = [];
    hh_temp = [];
    zz_temp = [];
    % 将分到某一类的stixel放在xx，yy中，即二维平面上
    for s = 1:Stixel_number
        if(cluster_flag(s) == f)
            xx = [xx Stixel_result_x(s)];
            yy = [yy Stixel_result_y(s)];
            hh = [hh Stixel_result_z(s)];
            zz = [zz Stixel_result_h(s)];
        end
    end
    
%     p = polyfit(xx,yy,1);
%     plot(xx,yy,'o',xx,polyval(p,xx));
%     hold on;
    Width_minLine = 5;
    W_line = Width_minLine;
    cnt_i = 1;

    while (isempty(xx) == 0)
            
            [one_temp,number_exist] = size(xx);
            % 单独一个点的处理，要么结合到之前的直线中，如果无法结合就与相邻最近的stixel形成小的平面
            if (number_exist == 1)
                hold on;
                xx = [];
                yy = [];
                zz = [];
                hh = [];
                plot(xx_temp,yy_temp,'o');%,xx_temp,polyval(p,xx_temp));
                hold on;
                [one,size_temp] = size(xx_temp);
                if (size_temp>1)
                    plane_x_min = [plane_x_min xx_temp(1)];
                    plane_x_max = [plane_x_max xx_temp(size_temp)];
                    plane_y_min = [plane_y_min xx_temp(1)*p(1)+p(2)];
                    plane_y_max = [plane_y_max xx_temp(size_temp)*p(1)+p(2)];
                    plane_z_min = [plane_z_min min(zz_temp)];
                    plane_z_max = [plane_z_max max(hh_temp)];    % 此处应该是小于1，则等于0
                    plane_number = plane_number + 1;
                    line_k = [line_k p(1)];
                    line_b = [line_b p(2)];
                end
                % 仅有一个点，不足以形成平面，考虑采用连接上一个点的方式构成平面
                last_p = [];
                xx_temp = [];
                yy_temp = [];
                hh_temp = [];
                zz_temp = [];
                break;
                
            else
                if (number_exist>=Width_minLine)
                    W_line = Width_minLine;
                else
                    W_line = number_exist;
                end
                xx_temp = [xx_temp xx(1:W_line)];
                yy_temp = [yy_temp yy(1:W_line)];
                hh_temp = [hh_temp hh(1:W_line)];
                zz_temp = [zz_temp zz(1:W_line)];
                %p = polyfit(xx_temp,yy_temp,1);

                [one,size_temp] = size(xx_temp);
                error = zeros(size_temp,1);
                error_diff = zeros(size_temp,1);
                error_number = 0; % 记录误差数量，若误差数量较少则不需考虑（噪声）
                n = size_temp;
                
                if ((max(xx_temp)-min(xx_temp))<0.1)
                    p(1) = nan;
                    p(2) = nan;
                else
                    % 用最小二乘法拟合直线段
                    x_2=sum(xx_temp.^2);              % 求Σ(xi^2)
                    x_1=sum(xx_temp);                 % 求Σ(xi)
                    x_1y_1=sum(xx_temp.*yy_temp);     % 求Σ(xi*yi)
                    y_1=sum(yy_temp);                 % 求Σ(yi)

                    a=(n*x_1y_1-x_1*y_1)/(n*x_2-x_1*x_1);      %解出直线斜率b=(y1-a*x1)/n
                    b=(y_1-a*x_1)/n;                      %解出直线截距

                    p(1) = a;
                    p(2) = b;
                end
                average_x = mean(xx_temp);
                % 当直线斜率过大，直接计算x轴坐标差即可
                if ((p(1)>2000)||(isnan(p(1))))
                     for e = 1:size_temp
                        error(e) = (xx_temp(e) - average_x);
                    end
                else
                    for e = 1:size_temp
                        error(e) = (p(1)*xx_temp(e)+p(2)-yy_temp(e))/sqrt(1+p(1)^2);                      
                    end
                end
                % 统计障碍物的凹凸性
                line_concavity = zeros(size_temp,1);
                for e = 1:W_line-1
                    if (error(e)*error(e+1)<0)
                        line_concavity(e) = 1;
                    end
                end
                sum_concavity = sum(line_concavity);
                if (sum_concavity>(3))
                    concavity_flag = 1;
                else
                    concavity_flag = 0;
                end
                for e = 1:size_temp
                    error(e) = abs(error(e));                      
                end
                [I,N] = max(error);
                if (max(error)<=0.2)
                    for delate = 1:W_line
                        xx(1) = [];
                        yy(1) = [];
                        zz(1) = [];
                        hh(1) = [];
                    end
                    last_p = p;
                else
                    % 填补凹凸
                    if (concavity_flag == 1)
                        finish_flag = 1;
                        while(finish_flag == 1)
                            finish_flag = 0;
                            [one,size_temp] = size(xx_temp);
                            delate_flag = zeros(size_temp,1);
                            for e = 2:size_temp-1
                                BA_vector_X = xx_temp(e) - xx_temp(e-1);
                                BA_vector_Y = yy_temp(e) - yy_temp(e-1);
                                BC_vector_X = xx_temp(e) - xx_temp(e+1);
                                BC_vector_Y = yy_temp(e) - yy_temp(e+1);
                                judge_concavity = BA_vector_X * BC_vector_Y - BA_vector_Y * BC_vector_X;
                                distance_AC = sqrt((xx_temp(e-1)-xx_temp(e+1))^2 +  ...
                                    (yy_temp(e-1)-yy_temp(e+1))^2);
                                if ((judge_concavity>0) && (distance_AC < 2))
                                    delate_flag(e) = 1;
                                    finish_flag = 1;
                                end
                            end
                            for e = size_temp-1:-1:2
                                if (delate_flag(e) == 1)
                                    xx_temp(e) = [];
                                    yy_temp(e) = [];
                                    hh_temp(e) = [];
                                    zz_temp(e) = [];
                                end
                            end
                        end
                        [one,size_temp] = size(xx_temp);
                        for e = 1:size_temp-1
                            p(1) = (yy_temp(e) - yy_temp(e+1))/(xx_temp(e) - xx_temp(e+1));
                            p(2) = (xx_temp(e)*yy_temp(e+1)-xx_temp(e+1)*yy_temp(e))/(xx_temp(e)-xx_temp(e+1));
                            
                            plot(xx_temp(e:e+1),yy_temp(e:e+1),'o',xx_temp(e:e+1),polyval(p,xx_temp(e:e+1)));
                            hold on;
                            % 记录平面的四个点坐标参数
                            [one,size_temp] = size(xx_temp);
                            plane_x_min = [plane_x_min xx_temp(e)];
                            plane_x_max = [plane_x_max xx_temp(e+1)];
                            plane_y_min = [plane_y_min xx_temp(1)*p(1)+p(2)];
                            plane_y_max = [plane_y_max xx_temp(size_temp)*p(1)+p(2)];
                            plane_z_min = [plane_z_min min(zz_temp)];
                            plane_z_max = [plane_z_max max(hh_temp)];     
                            plane_number = plane_number + 1;

                            last_p = [];                   
                            line_k = [line_k p(1)];
                            line_b = [line_b p(2)];
                        end
                        xx_temp = [];
                        yy_temp = [];
                        hh_temp = [];
                        zz_temp = [];    
                        for delate = 1:W_line
                            xx(1) = [];
                            yy(1) = [];
                            zz(1) = [];
                            hh(1) = [];
                        end
                    else
                        % 当直线段误差较大时出现转折点，计算转折点位置
                        turn_point = 0;
                        for e = 2:size_temp-1
                            if (error(e)>error(e-1)&&(error(e)>error(e+1))&&(error(e)>0.1))
                                turn_point = e;
                                break;
                            end
                            if (e == (size_temp-1))
                                average_error = 2 * mean(error);
                                for point_cnt = 2:size_temp
                                    if (error(point_cnt) > average_error)
                                        turn_point = max(point_cnt,1);
                                        break;
                                    end    
                                end
                            end
                        end
                        % 常规方法没有找到合适的折断点，即可能第一点误差较大，或特别特殊位置误差较大，设为1，单个点处理
                        if (turn_point == 0)
                            turn_point = 1;
                        end
                        % 此处修补误差，当转折点在3个以上W_line时，需要为xx恢复，可以重新设计xx与xx_temp关系
                        if ((size_temp - turn_point)>= W_line)
                            for add = 1:(size_temp - turn_point - W_line)
                               xx = [xx_temp(size_temp +1 - add - W_line) xx];
                               yy = [yy_temp(size_temp +1 - add - W_line) yy];
                               hh = [hh_temp(size_temp +1 - add - W_line) hh];
                               zz = [zz_temp(size_temp +1 - add - W_line) zz];                         
                            end
                        elseif (turn_point >=1)
                            for delate = 1:W_line - (size_temp - turn_point)
                                xx(1) = [];
                                yy(1) = [];
                                hh(1) = [];
                                zz(1) = [];                            
                            end
                        end
                        % 将转折点后的点从xx_temp中删除
                        for delate = 1:size_temp-turn_point
                            xx_temp(size_temp-delate + 1) = [];
                            yy_temp(size_temp-delate + 1) = [];
                            hh_temp(size_temp-delate + 1) = [];
                            zz_temp(size_temp-delate + 1) = [];
                        end

                        [one,size_temp] = size(xx_temp);
                        n = size_temp;
                        % 对更新过的点用最小二乘法拟合直线段
                        x_2=sum(xx_temp.^2);              % 求Σ(xi^2)
                        x_1=sum(xx_temp);                 % 求Σ(xi)
                        x_1y_1=sum(xx_temp.*yy_temp);     % 求Σ(xi*yi)
                        y_1=sum(yy_temp);                 % 求Σ(yi)
                        a=(n*x_1y_1-x_1*y_1)/(n*x_2-x_1*x_1);      %解出直线斜率b=(y1-a*x1)/n
                        b=(y_1-a*x_1)/n;                      %解出直线截距
                        p(1) = a;
                        p(2) = b;

                        if (isempty(xx_temp) == 0)

                            plot(xx_temp,yy_temp,'o',xx_temp,polyval(p,xx_temp));
                            hold on;
                            % 记录平面的四个点坐标参数
                            [one,size_temp] = size(xx_temp);
                            plane_x_min = [plane_x_min xx_temp(1)];
                            plane_x_max = [plane_x_max xx_temp(size_temp)];
                            plane_y_min = [plane_y_min xx_temp(1)*p(1)+p(2)];
                            plane_y_max = [plane_y_max xx_temp(size_temp)*p(1)+p(2)];
                            plane_z_min = [plane_z_min min(zz_temp)];
                            plane_z_max = [plane_z_max max(hh_temp)];     
                            plane_number = plane_number + 1;

                            last_p = [];
                            xx_temp = [];
                            yy_temp = [];
                            hh_temp = [];
                            zz_temp = [];                        
                            line_k = [line_k p(1)];
                            line_b = [line_b p(2)];
                        end
                    end
                end
            end
            
            if ((isempty(xx) == 1)&&(~isempty(xx_temp)))
                plot(xx_temp,yy_temp,'o',xx_temp,polyval(p,xx_temp));
                hold on;
                plane_x_min = [plane_x_min xx_temp(1)];
                plane_x_max = [plane_x_max xx_temp(size_temp)];
                plane_y_min = [plane_y_min xx_temp(1)*p(1)+p(2)];
                plane_y_max = [plane_y_max xx_temp(size_temp)*p(1)+p(2)];
                plane_z_min = [plane_z_min min(zz_temp)];
                plane_z_max = [plane_z_max max(hh_temp)];  
                plane_number = plane_number + 1;
                line_k = [line_k p(1)];
                line_b = [line_b p(2)];

                last_p = [];
                line_k = [line_k p(1)];
                line_b = [line_b p(2)];
                xx_temp = [];
                yy_temp = [];
                hh_temp = [];
                zz_temp = [];        
            end
        % 找出误差e的截断点，然后将线段分开，之前的分为一个，之后的分为一个
    end
    
end

figure(12);
% 绘制
for s = 1:Stixel_number
    plot3([Stixel_result_x(s) Stixel_result_x(s)],[Stixel_result_y(s) Stixel_result_y(s)], ...
        [Stixel_result_h(s) Stixel_result_z(s)], 'LineWidth',2);
    hold on;
    grid on
end

figure(15);
hold on;
grid on;
for p = 1:plane_number
    x1 = plane_x_min(p);
    x2 = plane_x_max(p);
    x3 = plane_x_max(p);
    x4 = plane_x_min(p);
    
    y1 = plane_y_min(p);
    y2 = plane_y_max(p);
    y3 = plane_y_max(p);
    y4 = plane_y_min(p);
    
    z1 = plane_z_max(p);
    z2 = plane_z_max(p);
    z3 = plane_z_min(p);
    z4 = plane_z_min(p);
    hold on;
    fill3([x1 x2 x3 x4],[ y1 y2 y3 y4],[z1 z2 z3 z4],'b');
    alpha(.5);
    %shading interp
end

%% 相邻平面再次合并，若无法合并则填补缝隙
delate_plane_flag = zeros(plane_number,1);
for p = 1:1:(plane_number-1)
    x1 = plane_x_max(p);
    x2 = plane_x_min(p+1);
    
    y1 = plane_y_max(p);
    y2 = plane_y_min(p+1);

    % 相邻平面的最小距离
    distance_12  = sqrt((x1-x2)^2+(y1-y2)^2);
    if (distance_12 < Width_safe)
        k1 = line_k(p);
        k2 = line_k(p+1);
        cos_k1k2 = (1+k1*k2)/(sqrt(1+k1^2)*sqrt(1+k2^2));
        theta_k1k2 = acos(cos_k1k2);
        
        plane_k1k2_x = plane_x_max(p) - plane_x_min(p+1);
        plane_k1k2_y = plane_y_max(p) - plane_y_min(p+1);
        commom_plane = ((plane_k1k2_x * 1) + (plane_k1k2_y*k1))/(sqrt(plane_k1k2_x^2+plane_k1k2_y^2)*sqrt(k1^2+k2^2));
        % 若两个平面近似共面，则直接合并两个平面
        if ((commom_plane<0.1)&&(theta_k1k2<0.1)&&(plane_z_min(p)-plane_z_min(p+1))<0.5&&(plane_z_max(p)-plane_z_max(p+1))<0.5) 
            % 满足共面
            plane_x_max(p) = plane_x_max(p+1);
            plane_y_max(p) = plane_y_max(p+1);
            plane_z_min(p) = min(plane_z_min(p),plane_z_min(p+1));
            plane_z_max(p) = max(plane_z_max(p),plane_z_max(p+1));
            
            delate_plane_flag(p+1) = 1;
        else
            intersect_x = (line_b(p)-line_b(p+1))/(line_k(p+1)-line_k(p));
            intersect_y = (line_k(p)*line_b(p+1)-line_k(p+1)*line_b(p))/(line_k(p)-line_k(p+1));
            distance_intersect1 = sqrt((intersect_x-plane_x_max(p))^2 + (intersect_y-plane_y_max(p))^2);
            distance_intersect2 = sqrt((intersect_x-plane_x_min(p+1))^2 + (intersect_y-plane_y_min(p+1))^2);
            if (distance_intersect1<6*distance_12)||(distance_intersect2<6*distance_12)
                plane_x_max(p) = intersect_x;
                plane_x_min(p+1) = intersect_x;
                plane_y_max(p) = intersect_y;
                plane_y_min(p+1) = intersect_y;
                if ((plane_z_min(p)-plane_z_min(p+1))<0.5&&(plane_z_max(p)-plane_z_max(p+1))<0.5)
                    plane_z_min(p) = min(plane_z_min(p),plane_z_min(p+1));
                    plane_z_max(p) = max(plane_z_max(p),plane_z_max(p+1));
                    plane_z_min(p) = min(plane_z_min(p),plane_z_min(p+1));
                    plane_z_max(p) = max(plane_z_max(p),plane_z_max(p+1));
                end
            else 
                plane_x_min = [plane_x_min plane_x_max(p)];
                plane_x_max = [plane_x_max plane_x_min(p+1)];
                plane_y_min = [plane_y_min plane_y_max(p)];
                plane_y_max = [plane_y_max plane_y_min(p+1)];
                plane_z_min = [plane_z_min min(plane_z_min(p),plane_z_min(p+1))];
                plane_z_max = [plane_z_max max(plane_z_max(p),plane_z_max(p+1))];  
                plane_number = plane_number + 1;
            end

        end
    end
    
end

figure(16);
hold on;
grid on;
delate_panle = zeros(plane_number,1);
size_delate_number = size(delate_plane_flag);
for p = 1:size_delate_number(1)
    delate_panle(p) = delate_plane_flag(p);
end
for p = 1:plane_number
    if (delate_panle(p) == 0)
        x1 = plane_x_min(p);
        x2 = plane_x_max(p);
        x3 = plane_x_max(p);
        x4 = plane_x_min(p);

        y1 = plane_y_min(p);
        y2 = plane_y_max(p);
        y3 = plane_y_max(p);
        y4 = plane_y_min(p);

        z1 = plane_z_max(p);
        z2 = plane_z_max(p);
        z3 = plane_z_min(p);
        z4 = plane_z_min(p);
        hold on;
        fill3([x1 x2 x3 x4],[ y1 y2 y3 y4],[z1 z2 z3 z4],'b');
        alpha(.5);
        %shading interp
    end
end
% hold on;
% fill3([-5 -5 5 5],[ 0 10 10 0],[0 0 0 0],'k');
% alpha(.5);
