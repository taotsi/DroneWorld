%function [] = KDE_stixel()
% pict ��¼���д˴�����֡����
clearvars -EXCEPT pict
close all;clc
% ��¼�ϰ�������ȡ�Ľ������x,y��Ϊ��άӳ�����꣬hΪ�ϰ����¶˵���������߶ȣ�zΪ�϶���������߶�
Stixel_result_x = [];
Stixel_result_y = [];
% ����λ��(x,y)
Stixel_result_z = [];
Stixel_result_h = [];
%��¼��stixel�Ƿ񳬳���Ұ��������Ϊ1 ���������������ֱ��¼�Ϸ����·�������Ұ
Beyond_view_flag = [];
below_view_flag = [];
% ��¼�ϰ�����ȡ����
Stixel_number = 0;
D = imread('4disparity_screenshot_12.09.2019.png');
D = rgb2gray(D);
figure(2);
[rows,cols] = size(D);
imshow(D);
D = 64*im2double(D);
%w2=fspecial('average',[1 3]);
%D=imfilter(D,w2,'replicate');
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%��ȡtxt�д洢�����˻�״̬����
%w2=fspecial('average',[5 5]);
%D=imfilter(D,w2,'replicate');
uav_x = 0;
uav_y = 0;
uav_z = 1.73;


RGBshow = imread('0000000092.png');
figure(20);hold on;
imshow(RGBshow);
% ����������������������任�ı���,�˴��ĽǶ���Ҫȷ��������
CosValue = 1;
SinValue = 0;


%for n = 200:1:201
%��ͼѡ��������
w=796;
figure(3);
plot(D(1:rows,w));
%axis([0 375 0 260])
%����ĳһ������Ӳ�
A_plot = D(1:rows,w);
%A = 10*A;
% ����stixel���
stixel_width = 7;
% ����ֱ��ͼ��ͳ�Ʒֱ���
width=64;


%�����������
fx = 984.2439;
fy = 980.8;
v0 = 233.2;
u0 = 690;
Baseline = 0.25;


% �����Ƿ������ݣ���Ҫ�Լ��޶���Χ
% ��������������Ӳ������С�ֱ��ʺ����̽����뷶Χ
Z_min = 0.5;
Z_max = 20;
Disparity_min = fx * Baseline / Z_max;
Disparity_max = fx * Baseline / Z_min;
% ����������Ч�߶ȣ���С����ĸ߶�
detal_y = 0.3;
sitxel_cnt = 1;
% ���ú��ܶȹ��ƵĿ�� �˴��������С����������������������stixel�����΢�仯�޷������ֽ���
Kernel_width = 1000;

figure(4);
[x,N] = hist(A_plot,width);
bar(x);
    
%���ܶȹ���

W5_gauss = [0.00170354, 0.109878, 0.440655, 0.109878, 0.0017035];
W7_gauss = [0.00038771, 0.01330373, 0.11098164, 0.22508352, 0.11098164, 0.01330373, 0.00038771];
W9_guass = [0.0561    0.1979    0.4868    0.8354    1.0001    0.8354    0.4868    0.1979    0.0561];
W17_guass = [0.0561    0.1103    0.1979    0.3247    0.4868    0.6670    0.8354    0.9561    1.0001    0.9561 0.8354    0.6670    0.4868    0.3247    0.1979    0.1103    0.0561];
W5_trian = [0.2 0.6 1 0.6 0.2];
W9_trian = [0.05 0.1 0.2 0.4 0.6 0.8 0.6 0.4 0.2 0.1 0.05];
W13_trian = [0.1 0.3 0.5 0.7 0.9 1.1 1.3 1.1 0.9 0.7 0.5 0.3 0.1];
W19_trian = [0.1 0.2 0.3 0.4 0.5 0.6 .07 0.8 0.9 1 0.9 0.8 0.7 0.6 0.5 0.4 0.3 0.2 0.1];
W25_trian = [0.1 0.2 0.3 0.4 0.5 0.6 .07 0.8 0.9 1 1.1 1.2 1.3 1.4 1.3 1.2 1.1 1 0.9 0.8 0.7 0.6 0.5 0.4 0.3 0.2 0.1];

% ���ܶȹ��Ƶķֱ���
[f,xi]=ksdensity(A_plot);
figure(5);
plot(f);
%%
    % ���Ƶ��Բ���
    clear A;
    A = [];
    KDE_D = zeros(Kernel_width,1);
    u = w;
    for v =1:rows
        if ((D(v,u)>Disparity_min)&&(D(v,u)<Disparity_max))
            A = [A D(v,u)];
            kde_cnt = round(D(v,u) * Kernel_width / Disparity_max);
            for k = -8:8
                if ((k + kde_cnt) > 1)
                    % �˴� -1 ����Ҫԭ���� 
                    KDE_D(k + kde_cnt) = KDE_D(k + kde_cnt) + W17_guass(k+9);
                end
            end
        end
    end
    figure(6);
    plot(KDE_D);
    
%     % ������̫����Բ���IIR�˲�������ë������
%     B = zeros(3,1);
%     A = zeros(3,1);
%     B(1)=1;B(2)=2;B(3)=1;
% 	A(1)=1;A(2)=-0.837000;A(3)=0.42398;
% 	Gain=0.146747;
%     w_x = zeros(3,1);
%     w_y = zeros(3,1);
%     KDE_filter = zeros(Kernel_width,1);
% 	for i = 1:1:Kernel_width
% 		w_x(1)=KDE_D(i);
% 		w_y(1)=(B(1)*w_x(1)+B(2)*w_x(2)+B(3)*w_x(3))*Gain-w_y(2)*A(2)-w_y(3)*A(3);
% 		KDE_filter(i)=w_y(1)/A(1);
% 		w_x(3)=w_x(2);w_x(2)=w_x(1);
% 		w_y(3)=w_y(2);w_y(2)=w_y(1);
%     end
%     
%     figure(7);
%     plot(KDE_filter);
    
    
figure(20);hold on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ��ȡ�ϰ�����
for u = 3:stixel_width:cols
    clear A;
    A = [];
    % ������Ӧ���ܶȿ�ȵĴ洢
    KDE_D = zeros(Kernel_width,1);
    % �˳��Ӳ��С���������100�׵���Ϣ
    % ���ø����ܶȹ���ͳ�Ʒֲ�
    for v =1:rows
        if ((D(v,u)>Disparity_min)&&(D(v,u)<Disparity_max))
            A = [A D(v,u)];
            kde_cnt = round(D(v,u) * Kernel_width / Disparity_max);
            for k = -8:8
                if ((k + kde_cnt) > 0)
                    KDE_D(k + kde_cnt) = KDE_D(k + kde_cnt) + W17_guass(k+9);
                end
            end
        end
    end
    
    % �ҳ����壬������һ����ֵ50cm�߶ȵ�����
    % ͨ�����ܶȹ��ƺ����ҳ��������Ⱦ�ֵ�ͷ���
    Object_cnt = 0;
    Object_mean = zeros(6,1);
    Object_depth_value = zeros(6,1);
    Object_detla = zeros(6,1);
    for t=2:Kernel_width-1
        % ��Ҫ����ȷ�еĽ��
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
    % ������������Object_cnt��
    for o=1:Object_cnt
        detla_cnt = 1;
        while(( 0.7 * KDE_D(Object_mean(o))  <  KDE_D( max(Object_mean(o)-detla_cnt ,1))) || ( 0.7 * KDE_D(Object_mean(o))  <  KDE_D( max(Object_mean(o) + detla_cnt,1))) )
            detla_cnt = detla_cnt + 1;
        end
        % �˴���ϵ�������ϰ������õ��Ŀ�Ⱦ���������Դ�һЩ
        Object_detla(Object_cnt) = 1 * detla_cnt * Disparity_max / Kernel_width;
    end
    
    % ����detal_y(30cm)�ĸ߶��ڶ�Ӧ�Ӳ��ϵ����ظ߶ȣ���Ϊ����ɸѡ����Сֵ��
    %detal_v = detal_y * Dis_maxnumber1 /Baseline;
    
    v=1;
    % ѡ����������������жϻ��˲�
    choose_case = 1;
    % �ж��Ƿ�������
    judge_object = 1;
    % �����˲�����
    filter_object = 2;
    % ��ǰ�������Ⱦ�ֵ�ͷ���
    Disparity_temp = 0;
    Detla_temp = 0;
    % ��¼�������������������ֵ���ж�
    judge_cnt = 0;
    
    clear A;
    A = D(1:rows,u);
    while (v<=rows) 
        
        switch choose_case
            %ѡ���ж��Ƿ����������
            case judge_object
                for o = 1:Object_cnt
                    if (abs(A(v)-Object_depth_value(o)) < Object_detla(o))
                        % ��������Ȼ��Ӳ���ĳһ�����Ӳ�������һ����Χ��ʱ�������˲�ģʽ
                        Disparity_temp = Object_depth_value(o);
                        Detla_temp = Object_detla(o);
                        choose_case = filter_object;
                        v = v - 1;
                    end
                end
                v = v + 1;
            %�������ڸ���������ظ���
            case filter_object
                h_filter = ceil(Disparity_temp * Kernel_width / Disparity_max);
                if (abs(A(v)-Disparity_temp) < Detla_temp)
                    judge_cnt =judge_cnt + 1;
                    v = v + 1;
                    if (v>=(rows))
                        line([u,u],[v,v-judge_cnt],'linestyle','-', 'LineWidth',2);
                        %rectangle('position',[u,rows-v,stixel_width,judge_cnt],'facecolor','g');
                        x_temp = (u-u0) * Baseline / Disparity_temp;
                        % ����ĸ߶�����Ϊ��h��3�·�
                        h_temp = -(fx * SinValue + (v-judge_cnt-v0) * CosValue) * Baseline / Disparity_temp + uav_z;
                        % �ϰ���Ͷ˵�������
                        y_temp = -(fx * SinValue + (v-v0) * CosValue) * Baseline / Disparity_temp + uav_z;
                        z_temp = (fx * CosValue - (v-judge_cnt-v0) * SinValue) * Baseline / Disparity_temp;
                        % �����������λ�ã�ת������������ϵ��
                        Position_camera = [x_temp z_temp h_temp 1];
                       
                        Position_world = Position_camera';
                        x_world = Position_world(1);
                        y_world = Position_world(2);
                        z_world = Position_world(3);
                
                        if (y_temp<1)
                            y_temp = 0;
                        end
                        if (abs(x_world)<5)
                             if ((h_temp>0)&&(abs(z_world-y_temp)>1))
                                Stixel_result_x = [Stixel_result_x x_world];
                                Stixel_result_y = [Stixel_result_y y_world];
                                Stixel_result_z = [Stixel_result_z z_world];
                                Stixel_result_h = [Stixel_result_h y_temp];
                                Stixel_number = Stixel_number + 1;
                            end                           
                        end

                        v = v + 1;
                        judge_cnt = 0;
                    end
                elseif (judge_cnt>20)
                    tempcnt = 0;
                    for h=v:min(v+h_filter,rows)
                        if (abs(A(h)-Disparity_temp) < Detla_temp)
                            tempcnt = tempcnt + 1;
                        end
                    end
                    if (tempcnt <= (h_filter/2))
                        line([u,u],[v,v-judge_cnt],'linestyle','-', 'LineWidth',2);
                        %rectangle('position',[u,rows-v,stixel_width,judge_cnt],'facecolor','g');
                        x_temp = (u-u0) * Baseline / Disparity_temp;
                        % ����ĸ߶�����Ϊ��h��3�·�  y_temp �������صͶ�  h_temp���������ϰ��ﶥ��
                        h_temp = -(fx * SinValue + (v-judge_cnt-v0) * CosValue) * Baseline / Disparity_temp + uav_z;
                        % �ϰ���Ͷ˵�������
                        y_temp = -(fx * SinValue + (v-v0) * CosValue) * Baseline / Disparity_temp + uav_z;
                        z_temp = (fx * CosValue - (v-judge_cnt-v0) * SinValue) * Baseline / Disparity_temp;
                        Position_camera = [x_temp z_temp h_temp 1];
                
                        Position_world = Position_camera';
                        x_world = Position_world(1);
                        y_world = Position_world(2);
                        z_world = Position_world(3);
                        if (y_temp<1)
                            y_temp = 0;
                        end
                        if (abs(x_world)<5)
                             if ((h_temp>0)&&(abs(z_world-y_temp)>1))
                                Stixel_result_x = [Stixel_result_x x_world];
                                Stixel_result_y = [Stixel_result_y y_world];
                                Stixel_result_z = [Stixel_result_z z_world];
                                Stixel_result_h = [Stixel_result_h y_temp];
                                Stixel_number = Stixel_number + 1;
                            end                           
                        end
                        judge_cnt = 0;
                        choose_case = judge_object;
                    else 
                        v = v + h_filter;
                    end
                    
                else
                    choose_case = judge_object;
                    judge_cnt = 0;                  
                end               
        end      
    end
    if (u>80)
    end
end


figure(11);
% ����
for s = 1:Stixel_number
    plot3([Stixel_result_x(s) Stixel_result_x(s)],[Stixel_result_y(s) Stixel_result_y(s)], ...
        [Stixel_result_h(s) Stixel_result_z(s)], 'LineWidth',2);
    hold on;
    grid on
end

%% �������
cluster_flag = [1];
cluster_number = 1;
plane_number = 0;
% ��¼������ĳһ�������
result_cluster_tempx = [Stixel_result_x(1)];
result_cluster_tempy = [Stixel_result_y(1)];
result_cluster_temph = [Stixel_result_h(1)];
result_cluster_tempz = [Stixel_result_z(1)];
% �˴������㷨���Կ���ͨ��ÿ��������ػ�ȡ������������о��ࡣ

for p = 2:Stixel_number
    for c = 1:1:cluster_number
        cluster_center_x = result_cluster_tempx(c);
        cluster_center_y = result_cluster_tempy(c);
        cluster_center_h = result_cluster_temph(c);
        cluster_center_z = result_cluster_tempz(c);
        distance_xy = sqrt(((Stixel_result_x(p)-cluster_center_x)^2+(Stixel_result_y(p)-cluster_center_y)^2 ));
        distance_z = 0;%abs(Stixel_result_h(p) - cluster_center_h); % z����Ĳ��죬���Ͷ˵ľ���
        distance_h = 0;%abs(Stixel_result_z(p) - cluster_center_z); % �߶ȵĲ���
        if ((distance_xy<2)&&(distance_z<1)&&(distance_h<1))
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


Height_safe = 2;
Width_safe  = 2;

%��x,y��Ϊ��άӳ�����꣬hΪ�ϰ����¶˵���������߶ȣ�zΪ�϶���������߶�
%% �߶��˲�
for f = 1:cluster_number
    Bef_filter_xx = [];
    Bef_filter_yy = [];
    Bef_filter_zz = [];
    Bef_filter_hh = [];
    
    % ���ֵ�ĳһ���stixel����xx��yy�У�����άƽ����
    serial_number_stixel = [];
    for s = 1:Stixel_number
        if(cluster_flag(s) == f)
            Bef_filter_xx = [Bef_filter_xx Stixel_result_x(s)];
            Bef_filter_yy = [Bef_filter_yy Stixel_result_y(s)];
            Bef_filter_hh = [Bef_filter_hh Stixel_result_z(s)];    % ת���hh���߶ȣ����Ϸ�����
            Bef_filter_zz = [Bef_filter_zz Stixel_result_h(s)];    % ת���zz���ײ�����
            serial_number_stixel = [serial_number_stixel s];
        end
    end
    % ��ȡ�����ڲ��ĸ߶����ֵ����Сֵ�����ڹ��ˡ�
    Zmax_temp = max(Bef_filter_hh);
    Zmin_temp = min(Bef_filter_zz);
    % ��ÿ�������ڲ�stixel�ĸ߶Ȳ���
    BefCluster_number = size(Bef_filter_xx);
    % �ֱ��¼�������ϲ��ֺ��²���
    Complement_Bef_z1 = zeros(BefCluster_number(2),2);
    Complement_Bef_z2 = zeros(BefCluster_number(2),2);
    % �������Ұ���޵��½���������stixelƫ��
    max_beyongd_temp = 0;
    if(~isempty(Beyond_view_flag))
        for s = 1:BefCluster_number(2)
            if (Beyond_view_flag(s) == 1)
                if (Bef_filter_hh(s) > max_beyongd_temp)
                    max_beyongd_temp = Bef_filter_hh(s);
                end
            end
        end
        % �Ը߶ȳ�����Ұ�Ľ��������
        for s = 1:Stixel_number       
            if(cluster_flag(s) == f)
                if (Beyond_view_flag(s) == 1)
                    Stixel_result_z(s) = max_beyongd_temp;    
                end
            end
        end
        
        % �����ڴ�������ݽ��и߶���Ұ���޲����������
        for s = 1:BefCluster_number(2)
            if (Beyond_view_flag(s) == 1)
                Bef_filter_hh(s) = max_beyongd_temp;
            end
        end
    end
    
    % �������Ұ���޵��½���������stixelƫ��
    min_beyongd_temp = Inf;
    if(~isempty(below_view_flag))
        for s = 1:BefCluster_number(2)
            if (below_view_flag(s) == 1)
                if (Bef_filter_hh(s) < min_beyongd_temp)
                    min_beyongd_temp = Bef_filter_hh(s);
                end
            end
        end
    % �Ը߶ȳ�����Ұ�Ľ��������
    
        for s = 1:Stixel_number
            if(cluster_flag(s) == f)
                if (below_view_flag(s) == 1)
                    Stixel_result_z(s) = min_beyongd_temp;    
                end
            end
        end
        
        % �����ڴ�������ݽ��и߶���Ұ���޲����������
        for s = 1:BefCluster_number(2)
            if (below_view_flag(s) == 1)
                Bef_filter_zz(s) = min_beyongd_temp;
            end
        end
    end


    % �����Ұ���޵��µ�ƫ���ⲹ����
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
    i = 1;
    while ( i < BefCluster_number(2)-1 ) 
        test_zmax = Complement_Bef_z1(i,1);
        test_zmin = Complement_Bef_z1(i,2);
        if ((test_zmax-test_zmin)>Height_safe)
            for j = i+1:BefCluster_number(2)
                % ���������и߶ȸ�ֵ
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
                        % ���м���ִ������˻��ɰ�ȫͨ�е�����ʱ����ͬһ������в�֡�
                        % ��������stixel���в�֣��õ��µķ���
                        % ��¼�Ϸ����·��Ƿ��ֿ�
                        clustering_add_cnt1 = 0;
                        clustering_add_cnt2 = 0;
                        for w = i:j
                            if (((Complement_Bef_z1(w,1)>=test_zmax)&&(Complement_Bef_z1(w,2)<=test_zmin))||  ...
                                ((Complement_Bef_z2(w,1)>=test_zmax)&&(Complement_Bef_z2(w,2)<=test_zmin)))
                                cluster_flag(serial_number_stixel(w)) = cluster_number + 1;
                                clustering_add_cnt1 = 1;
                            end
                        end
                        %�ܵľ��෢���ı�
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
                        % ���м���ִ������˻��ɰ�ȫͨ�е�����ʱ����ͬһ������в�֡�
                        % ��������stixel���в�֣��õ��µķ���
                        % ��¼�Ϸ����·��Ƿ��ֿ�
                        clustering_add_cnt1 = 0;
                        clustering_add_cnt2 = 0;
                        for w = i:j
                            if (((Complement_Bef_z1(w,1)>=test_zmax)&&(Complement_Bef_z1(w,2)<=test_zmin))||  ...
                                ((Complement_Bef_z2(w,1)>=test_zmax)&&(Complement_Bef_z2(w,2)<=test_zmin)))
                                cluster_flag(serial_number_stixel(w)) = cluster_number + 1;
                                clustering_add_cnt1 = 1;
                            end
                        end
                        %�ܵľ��෢���ı�
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
                % ���������и߶ȸ�ֵ
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
                        % ���м���ִ������˻��ɰ�ȫͨ�е�����ʱ����ͬһ������в�֡�
                        % ��������stixel���в�֣��õ��µķ���
                        % ��¼�Ϸ����·��Ƿ��ֿ�
                        clustering_add_cnt1 = 0;
                        clustering_add_cnt2 = 0;
                        for w = i:j
                            if (((Complement_Bef_z1(w,1)>=test_zmax)&&(Complement_Bef_z1(w,2)<=test_zmin))||  ...
                                ((Complement_Bef_z2(w,1)>=test_zmax)&&(Complement_Bef_z2(w,2)<=test_zmin)))
                                cluster_flag(serial_number_stixel(w)) = cluster_number + 1;
                                clustering_add_cnt1 = 1;
                            end
                        end
                        %�ܵľ��෢���ı�
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
                        % ���м���ִ������˻��ɰ�ȫͨ�е�����ʱ����ͬһ������в�֡�
                        % ��������stixel���в�֣��õ��µķ���
                        % ��¼�Ϸ����·��Ƿ��ֿ�
                        clustering_add_cnt1 = 0;
                        clustering_add_cnt2 = 0;
                        for w = i:j
                            if (((Complement_Bef_z1(w,1)>=test_zmax)&&(Complement_Bef_z1(w,2)<=test_zmin))||  ...
                                ((Complement_Bef_z2(w,1)>=test_zmax)&&(Complement_Bef_z2(w,2)<=test_zmin)))
                                cluster_flag(serial_number_stixel(w)) = cluster_number + 1;
                                clustering_add_cnt1 = 1;
                            end
                        end
                        %�ܵľ��෢���ı�
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

%% �߶��˲� �� ��ͬһ���ĵĸ߶Ƚ���ͳһ
for f = 1:cluster_number
    serial_number_stixel = [];
    filter_hh = [];
    filter_zz = [];
    for s = 1:Stixel_number
        if(cluster_flag(s) == f)
            filter_hh = [filter_hh Stixel_result_z(s)];    % ת���hh���߶ȣ����Ϸ�����
            filter_zz = [filter_zz Stixel_result_h(s)];    % ת���zz���ײ�����
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
    % �Ը߶ȳ�����Ұ�Ľ��������
    for s = 1:Stixel_number
        if(cluster_flag(s) == f)
            Stixel_result_z(s) = max__temp;    
            Stixel_result_h(s) = min__temp;  
        end
    end

end

%% ƽ����ȡ
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
    % ���ֵ�ĳһ���stixel����xx��yy�У�����άƽ����
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
    Width_minLine = 10;
    W_line = Width_minLine;
    cnt_i = 1;
    line_k = [];
    line_b = [];
    while (isempty(xx) == 0)
            
            [one_temp,number_exist] = size(xx);
            % ����һ����Ĵ���Ҫô��ϵ�֮ǰ��ֱ���У�����޷���Ͼ������������stixel�γ�С��ƽ��
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
                    plane_y_min = [plane_y_min yy_temp(1)];
                    plane_y_max = [plane_y_max yy_temp(size_temp)];
                    plane_z_min = [plane_z_min min(zz_temp)];
                    plane_z_max = [plane_z_max max(hh_temp)];    % �˴�Ӧ����С��1�������0
                    plane_number = plane_number + 1;
                end
                % ����һ���㣬�������γ�ƽ�棬���ǲ���������һ����ķ�ʽ����ƽ��
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
                error_number = 0; % ��¼�������������������������迼�ǣ�������
                n = size_temp;
                
                if ((max(xx_temp)-min(xx_temp))<0.1)
                    p(1) = nan;
                    p(2) = nan;
                else
                    % ����С���˷����ֱ�߶�
                    x_2=sum(xx_temp.^2);              % ��(xi^2)
                    x_1=sum(xx_temp);                 % ��(xi)
                    x_1y_1=sum(xx_temp.*yy_temp);     % ��(xi*yi)
                    y_1=sum(yy_temp);                 % ��(yi)

                    a=(n*x_1y_1-x_1*y_1)/(n*x_2-x_1*x_1);      %���ֱ��б��b=(y1-a*x1)/n
                    b=(y_1-a*x_1)/n;                      %���ֱ�߽ؾ�

                    p(1) = a;
                    p(2) = b;
                end
                average_x = mean(xx_temp);
                % ��ֱ��б�ʹ���ֱ�Ӽ���x��������
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
                if (max(error)<0.2)
                    for delate = 1:W_line
                        xx(1) = [];
                        yy(1) = [];
                        zz(1) = [];
                        hh(1) = [];
                        
                    end
                    last_p = p;
                else
                    % ��ֱ�߶����ϴ�ʱ����ת�۵㣬����ת�۵�λ��
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
                    % ���淽��û���ҵ����ʵ��۶ϵ㣬�����ܵ�һ�����ϴ󣬻��ر�����λ�����ϴ���Ϊ1�������㴦��
                    if (turn_point == 0)
                        turn_point = 1;
                    end
                    % �˴��޲�����ת�۵���3������W_lineʱ����ҪΪxx�ָ��������������xx��xx_temp��ϵ
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
                    % ��ת�۵��ĵ��xx_temp��ɾ��
                    for delate = 1:size_temp-turn_point
                        xx_temp(size_temp-delate + 1) = [];
                        yy_temp(size_temp-delate + 1) = [];
                        hh_temp(size_temp-delate + 1) = [];
                        zz_temp(size_temp-delate + 1) = [];
                    end
                    
                    [one,size_temp] = size(xx_temp);
                    n = size_temp;
                    % �Ը��¹��ĵ�����С���˷����ֱ�߶�
                    x_2=sum(xx_temp.^2);              % ��(xi^2)
                    x_1=sum(xx_temp);                 % ��(xi)
                    x_1y_1=sum(xx_temp.*yy_temp);     % ��(xi*yi)
                    y_1=sum(yy_temp);                 % ��(yi)
                    a=(n*x_1y_1-x_1*y_1)/(n*x_2-x_1*x_1);      %���ֱ��б��b=(y1-a*x1)/n
                    b=(y_1-a*x_1)/n;                      %���ֱ�߽ؾ�
                    p(1) = a;
                    p(2) = b;
                
                    if (isempty(xx_temp) == 0)
                        
                        plot(xx_temp,yy_temp,'o',xx_temp,polyval(p,xx_temp));
                        hold on;
                        % ��¼ƽ����ĸ����������
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
                        hh_temp = [];
                        zz_temp = [];                        
                        line_k = [line_k p(1)];
                        line_b = [line_b p(2)];
                    end
                end
            end
            
            if ((isempty(xx) == 1))
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
                hh_temp = [];
                zz_temp = [];        
            end
        % �ҳ����e�Ľضϵ㣬Ȼ���߶ηֿ���֮ǰ�ķ�Ϊһ����֮��ķ�Ϊһ��
    end
    
end

figure(12);
% ����
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





