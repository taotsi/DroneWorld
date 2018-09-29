%function [] = KDE_stixel()
% pict 记录运行此处，多帧运行
clearvars -EXCEPT pict
close all;clc
% 记录障碍物列提取的结果，（x,y）为二维映射坐标，h为障碍物下端的世界坐标高度，z为上端世界坐标高度
%pict = 61; 
Stixel_result_x = [];
Stixel_result_y = [];
% 坐标位置(x,y)
Stixel_result_z = [];
Stixel_result_h = [];

Stixel_result_vb = [];
Stixel_result_vt = [];
% 记录障碍物提取个数
Stixel_number = 0;

% 读取并转换深度数据（数据原格式为物体像平面的距离，单位m）

Filename = strcat('Depth/',num2str(pict), '.pfm');
Picture_num = pict + 1;
pict = pict + 5;
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
        D(u,v) = 448.1328 * 0.25 / (DEPTH(u,v)) + 0*rand(1);
    end
end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%读取txt中存储的无人机状态数据
kk = 100;  % 读取100次
[UAV_x,UAV_y,UAV_z,UAV_vx,UAV_vy,UAV_vz,UAV_wx,UAV_wy,UAV_wz,UAV_qw,UAV_qx,UAV_qy,UAV_qz] = ...
    textread('state.txt','%f%f%f%f%f%f%f%f%f%f%f%f%f',200);
uav_x = UAV_x(Picture_num);
uav_y = UAV_y(Picture_num);
uav_z = UAV_z(Picture_num);
UAV_roll = zeros(200,1);
UAV_pitch = zeros(200,1);
UAV_yaw = zeros(200,1);
for q = 1:200
    q_w = UAV_qw(q);
    q_x = UAV_qx(q);
    q_y = UAV_qy(q);
    q_z = UAV_qz(q);
    UAV_roll(q) = atan2(2 * (q_y*q_z + q_w*q_x), q_w*q_w - q_x*q_x - q_y*q_y + q_z*q_z);
    UAV_pitch(q) = asin(-2 * (q_x*q_z - q_w*q_y));
    UAV_yaw(q) = atan2(2 * (q_x*q_y + q_w*q_z), q_w*q_w + q_x*q_x - q_y*q_y - q_z*q_z);
end
% 计算像素坐标向世界坐标变换的变量,此处的角度需要确定正负号
CosValue = cos(UAV_pitch(Picture_num));
SinValue = -sin(UAV_pitch(Picture_num));
figure(1);
% 旋转深度图像，校正roll角度，由于角度参数不准确，此处先不做修正....
if (abs(-180*UAV_roll(Picture_num)/pi) > 3)
    I_rotate = imrotate(D,-180*UAV_roll(Picture_num)/pi,'bilinear','crop');
    imshow(I_rotate);
    %D = I_rotate;
else
    imshow(D/16);
end

[rows,cols] = size(D);
%for n = 200:1:201
%绘图选择列坐标
w=31;  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
Baseline = 0.25;


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
            for k = -4:4
                if ((k + kde_cnt) > 1)
                    % 此处 -1 的主要原因是 
                    KDE_D(k + kde_cnt) = KDE_D(k + kde_cnt) + W9_guass(k+5);
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
    
    figure(7);
    plot(KDE_filter);
    
    
Filename1 = strcat('Image/',num2str(pict-5), '.png');
RGBshow = imread(Filename1);
figure(2);hold on;
imshow(RGBshow);

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
            %为提高速度，可以将下面否循环岔开
            for k = -4:4
                if ((k + kde_cnt) > 0)
                    KDE_D(k + kde_cnt) = KDE_D(k + kde_cnt) + W9_guass(k+5);
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
    for t=2:Kernel_width-1
        % 需要计算确切的结果
        %delt_y = (v1-v2)*baseline/disparity;
        if ((KDE_D(t)*Baseline) > (0.5 * t * Disparity_max / Kernel_width))
            if ((KDE_D(t)>KDE_D(t-1))&&(KDE_D(t)>KDE_D(t+1)))
                if ((KDE_D(t)>KDE_D(t-2))&&(KDE_D(t)>KDE_D(t+2)))
                    if ((KDE_D(t)>KDE_D(t-3))&&(KDE_D(t)>KDE_D(t+3)))
                        Object_cnt = Object_cnt + 1;
                        Object_mean(Object_cnt) = t;
                        Object_depth_value(Object_cnt) = t * Disparity_max / Kernel_width;
                    end
                end
            end
        end

    end
    % 检测物体的数量Object_cnt个
    for o=1:Object_cnt
        detla_cnt = 1;
        while(( 0.7 * KDE_D(Object_mean(o))  <  KDE_D( Object_mean(o)-detla_cnt )) && ( 0.7 * KDE_D(Object_mean(o))  <  KDE_D( Object_mean(o) + detla_cnt)) )
            detla_cnt = detla_cnt + 1;
        end
        Object_detla(Object_cnt) = detla_cnt * Disparity_max / Kernel_width;
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
                h_filter = ceil(1 * Disparity_temp / Baseline);
                if (abs(A(v)-Disparity_temp) < Detla_temp)
                    judge_cnt =judge_cnt + 1;
                    v = v + 1;
                    if (v>=(rows))
                     
                        Stixel_result_vb = [Stixel_result_vb (v-judge_cnt)];
                        Stixel_result_vt = [Stixel_result_vt (v)];
                        line([u,u],[v,v-judge_cnt],'linestyle','-', 'LineWidth',2);
                        %rectangle('position',[u,rows-v,stixel_width,judge_cnt],'facecolor','g');
                        sum_temp = 0;
                        for l=(v-judge_cnt+2):v-4
                            sum_temp = sum_temp + D(l,u);
                        end
                        Disparity_temp = sum_temp / (judge_cnt-7);
                        
                        x_temp = (u-u0) * Baseline / Disparity_temp;
                        % 计算的高度数据为的h轴3下方
                        h_temp = -(fx * SinValue + (v-judge_cnt-v0) * CosValue) * Baseline / Disparity_temp;
                        % 障碍物低端的纵坐标
                        y_temp = -(fx * SinValue + (v-v0) * CosValue) * Baseline / Disparity_temp - UAV_z(Picture_num);
                        if (y_temp<0.5)
                            y_temp = 0;
                        end
                        z_temp = (fx * CosValue - (v-judge_cnt-v0) * SinValue) * Baseline / Disparity_temp;
                        % 计算相机坐标位置，转换到世界坐标系下
                        Position_camera = [x_temp z_temp h_temp 1];
                        Transform_matric = [cos(UAV_yaw(Picture_num))  sin(UAV_yaw(Picture_num)) 0 UAV_y(Picture_num); ...
                                            -sin(UAV_yaw(Picture_num)) cos(UAV_yaw(Picture_num)) 0 UAV_x(Picture_num); ...
                                            0                          0                         1 -UAV_z(Picture_num)];
                  
                        Position_world = Transform_matric * Position_camera';
                        x_world = Position_world(1);
                        y_world = Position_world(2);
                        z_world = Position_world(3);
                
                        if (h_temp>0)
                            Stixel_result_x = [Stixel_result_x x_world];
                            Stixel_result_y = [Stixel_result_y y_world];
                            Stixel_result_z = [Stixel_result_z z_world];
                            Stixel_result_h = [Stixel_result_h y_temp];
                            Stixel_number = Stixel_number + 1;
                        end
                        v = v + 1;
                        judge_cnt = 0;
                    end
                elseif (judge_cnt>10)
                    tempcnt = 0;
                    for h=v:min(v+h_filter,rows)
                        if (abs(A(h)-Disparity_temp) < Detla_temp)
                            tempcnt = tempcnt + 1;
                        end
                    end
                    if (tempcnt <= 5)
                        Stixel_result_vb = [Stixel_result_vb (v-judge_cnt)];
                        Stixel_result_vt = [Stixel_result_vt (v)];
                        
                        sum_temp = 0;
                        for l=(v-judge_cnt+2):v-4
                            sum_temp = sum_temp + D(l,u);
                        end
                        Disparity_temp = sum_temp / (judge_cnt-7);
                        line([u,u],[v,v-judge_cnt],'linestyle','-', 'LineWidth',2);
                        %rectangle('position',[u,rows-v,stixel_width,judge_cnt],'facecolor','g');
                        x_temp = (u-u0) * Baseline / Disparity_temp;
                        % 计算的高度数据为的h轴3下方  y_temp 是列像素低端  h_temp是列像素障碍物顶端
                        h_temp = -(fx * SinValue + (v-judge_cnt-v0) * CosValue) * Baseline / Disparity_temp;
                        % 障碍物低端的纵坐标
                        y_temp = -(fx * SinValue + (v-v0) * CosValue) * Baseline / Disparity_temp - UAV_z(Picture_num);
                        if (y_temp<0.5)
                            y_temp = 0;
                        end
                        z_temp = (fx * CosValue - (v-judge_cnt-v0) * SinValue) * Baseline / Disparity_temp;
                        Position_camera = [x_temp z_temp h_temp 1];
                        Transform_matric = [cos(UAV_yaw(Picture_num))  sin(UAV_yaw(Picture_num)) 0 UAV_y(Picture_num); ...
                                            -sin(UAV_yaw(Picture_num)) cos(UAV_yaw(Picture_num)) 0 UAV_x(Picture_num); ...
                                            0                          0                         1 -UAV_z(Picture_num)];
                
                        Position_world = Transform_matric * Position_camera';
                        x_world = Position_world(1);
                        y_world = Position_world(2);
                        z_world = Position_world(3);

                        if (h_temp>0)
                            Stixel_result_x = [Stixel_result_x x_world];
                            Stixel_result_y = [Stixel_result_y y_world];
                            Stixel_result_z = [Stixel_result_z z_world];
                            Stixel_result_h = [Stixel_result_h y_temp];
                            Stixel_number = Stixel_number + 1;
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
    if (u>836)
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

%% 平面提取
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
        cluster_center_x = result_cluster_tempx(c);
        cluster_center_y = result_cluster_tempy(c);
        cluster_center_h = result_cluster_temph(c);
        cluster_center_z = result_cluster_tempz(c);
        distance_xy = sqrt(((Stixel_result_x(p)-cluster_center_x)^2+(Stixel_result_y(p)-cluster_center_y)^2 ));
        distance_z = 0;%abs(Stixel_result_h(p) - cluster_center_h); % z坐标的差异，即低端的距离
        distance_h = 0;%abs(Stixel_result_z(p) - cluster_center_z); % 高度的差异
        if ((distance_xy<2)&&(distance_z<2)&&(distance_h<2))
            cluster_flag = [cluster_flag c];
            result_cluster_tempx(cluster_number) = Stixel_result_x(p);
            result_cluster_tempy(cluster_number) = Stixel_result_y(p);
            result_cluster_temph(cluster_number) = Stixel_result_h(p);
            result_cluster_tempz(cluster_number) = Stixel_result_z(p);
            break;
        elseif (c == cluster_number)
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
    line_k = [];
    line_b = [];
    while (isempty(xx) == 0)
            
            [one_temp,number_exist] = size(xx);
            % 单独一个点的处理，要么结合到之前的直线中，如果无法结合就与相邻最近的stixel形成小的平面
            if (number_exist == 1)
                hold on;
                xx = [];
                yy = [];
                plot(xx_temp,yy_temp,'o');%,xx_temp,polyval(p,xx_temp));
                hold on;
                [one,size_temp] = size(xx_temp);
                if (size_temp>1)
                    plane_x_min = [plane_x_min xx_temp(1)];
                    plane_x_max = [plane_x_max xx_temp(size_temp)];
                    plane_y_min = [plane_y_min yy_temp(1)];
                    plane_y_max = [plane_y_max yy_temp(size_temp)];
                    plane_z_min = [plane_z_min min(zz_temp)];
                    plane_z_max = [plane_z_max max(hh_temp)];  
                    plane_number = plane_number + 1;
                end
                % 仅有一个点，不足以形成平面，考虑采用连接上一个点的方式构成平面
                last_p = [];
                xx_temp = [];
                yy_temp = [];
                
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
                        error(e) = abs(xx_temp(e) - average_x);
                    end
                else
                    for e = 1:size_temp
                        %error(e) = abs(yy_temp(e) - (p(1)*xx_temp(e)+p(2)));
                        error(e) = abs(p(1)*xx_temp(e)+p(2)-yy_temp(e))/sqrt(1+p(1)^2);
                        
                    end
                    for e = 2:size_temp
                        error_diff = error(e) - error(e-1);
                    end
                end
                [I,N] = max(error);
                if (max(error)<0.1)
                    for delate = 1:W_line
                        xx(1) = [];
                        yy(1) = [];
                    end
                    last_p = p;
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
                    if ((size_temp - turn_point)>=5)
                        for add = 1:(size_temp - turn_point - 5)
                           xx = [xx_temp(size_temp +1 - add - 5) xx];
                           yy = [yy_temp(size_temp +1 - add - 5) yy];
                        end
                    elseif (turn_point >=1)
                        for delate = 1:W_line - (size_temp - turn_point)
                            xx(1) = [];
                            yy(1) = [];
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
                        plane_y_min = [plane_y_min yy_temp(1)];
                        plane_y_max = [plane_y_max yy_temp(size_temp)];
                        plane_z_min = [plane_z_min min(zz_temp)];
                        plane_z_max = [plane_z_max max(hh_temp)];     
                        plane_number = plane_number + 1;

                        last_p = [];
                        xx_temp = [];
                        yy_temp = [];
                        line_k = [line_k p(1)];
                        line_b = [line_b p(2)];
                    end
                end
            end
            
            if ((isempty(xx) == 1)&&(isempty(xx_temp) == 0))
                plot(xx_temp,yy_temp,'o',xx_temp,polyval(p,xx_temp));
                hold on;
                plane_x_min = [plane_x_min xx_temp(1)];
                plane_x_max = [plane_x_max xx_temp(size_temp)];
                plane_y_min = [plane_y_min yy_temp(1)];
                plane_y_max = [plane_y_max yy_temp(size_temp)];
                plane_z_min = [plane_z_min min(zz_temp)];
                plane_z_max = [plane_z_max max(hh_temp)];  
                plane_number = plane_number + 1;

                last_p = [];
                xx_temp = [];
                yy_temp = [];

            end
        % 找出误差e的截断点，然后将线段分开，之前的分为一个，之后的分为一个
    end
    
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
end

