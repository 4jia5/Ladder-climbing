% 本程序完成了单一的Chebyshev结构的运动以及机器人外形的勾画
% 本程序可以用一对chebyshev机构完成多次攀爬
% 每次使用梳状抓手的哪一级step在程序里是确定的
% 在本程序的基础上可以开始考虑连杆尺寸的优化了
% 本程序由chebyshev_ladder_up_17而来，差别在于改了连杆和抓手尺寸
% 本程序改了comb gap


clear;
clc;

a=43.1888;
% 原始比例
% A is the wheel(p1p2),D is p1p4。 p1位于坐标原点
% p3p4为最短杆（C），p4为固定点，p3为链接最短杆和最长杆的点

K=1;  % K 为最长杆靠近动力一段和末段的长度比，K>1时末段长
A = 1.0741*a; B =2.7915*a; C = 2.7658*a; D = 2.1553*a; F =K*B;

L=230;    % L为多出来的连杆的长度
c=15;     % c为梳状抓手宽度
comb_gap=43;  % 梳状抓手间隔
comb_gap2=34;  % 梳状抓手间隔


% 设置机器人外轮廓的固定点坐标
R1 = [-A,0];  
R2 = [-A,150]; 
R3 = [1.1*A,D];
R4 = [1.1*A,-1.1*A];
R5 = [-A,-150];



% length parameters

ang_speed = -1/2 * pi;

delta_t=0.05;
T=16;

iit=T/delta_t+1;

t=0:delta_t:T;
t1=0.05:delta_t:T;
t2=0.1:delta_t:T;

% ang_speed = 2;
theta = ang_speed * t;

P1 = [0;0];        %p1为短杆（A杆）固定点
P4 = D * [0;1];    %p4为长杆（C杆）固定点


P2_x = A*cos(theta);       %theta为p1p2转过的角度（与x正方向夹角）
P2_y = A*sin(theta);
P2 = [P2_x; P2_y];    %p2为圆周转动点

E = sqrt(A^2 + D^2 - 2*A*D*cos(pi/2-theta));      % p2p4=E
alfa = asin(A*sin(pi/2-theta)./E);                % alfa为p1p4和p2p4夹角
beta = acos((E.^2 + C^2 - B^2)./(2*E*C));     %beta为p2p4和p3p4夹角
P3 = [C*sin(alfa+beta);D - C*cos(alfa+beta)];      %p3为最长杆中间点


P3_x = P3(1,:);
P3_y = P3(2,:);

P3_vx = diff(P3_x)./diff(t);
P3_vy = diff(P3_y)./diff(t);

P3_v = sqrt(P3_vx.^2 + P3_vy.^2);



% P5                   %p5为最长杆末端点
P5_x = P2_x + (1+K) .* (P3_x - P2_x);
P5_y = P2_y + (1+K) .* (P3_y - P2_y);


P5 = [P5_x; P5_y];
% P5_vx = (1+K) .* P3_vx;
% P5_vy = (1+K) .* P3_vy;
% P5_v = (1+K) .* P3_v;

P5_v =diff(P5);
P5_vy = diff (P5_y);
% P5

% 以下为chebychev多出来的那根连杆
P6_x=P5_x-L;
P6_y=P5_y;
P6 = [P6_x; P6_y];
% P6_vx = (1+K) .* P3_vx;
% P6_vy = (1+K) .* P3_vy;
% P6_v = (1+K) .* P3_v;

P6_vy = P5_vy;
P6_v = P5_v;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 以下为另一侧的连杆机构 q %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

theta_Q = ang_speed * t+pi;

Q1 = [0;0];        %p1为短杆（A杆）固定点
Q4 = D * [0;1];    %p4为长杆（C杆）固定点

Q2_x = A*cos(theta_Q);       %theta为p1p2转过的角度（与x正方向夹角）
Q2_y = A*sin(theta_Q);
Q2 = [Q2_x; Q2_y];    %p2为圆周转动点

E_Q = sqrt(A^2 + D^2 - 2*A*D*cos(pi/2-theta_Q));      % p2p4=E
alfa_Q = asin(A*sin(pi/2-theta_Q)./E_Q);                % alfa为p1p4和p2p4夹角
beta_Q = acos((E_Q.^2 + C^2 - B^2)./(2*E_Q*C));     %beta为p2p4和p3p4夹角

Q3 = [C*sin(alfa_Q+beta_Q);D - C*cos(alfa_Q+beta_Q)];      %p3为最长杆中间点
Q3_x = Q3(1,:);
Q3_y = Q3(2,:);
Q3_vx = diff(Q3_x)./diff(t);
Q3_vy = diff(Q3_y)./diff(t);
Q3_v = sqrt(Q3_vx.^2 + Q3_vy.^2);

% Q5                   %p5为最长杆末端点
Q5_x = Q2_x + (1+K) .* (Q3_x - Q2_x);
Q5_y = Q2_y + (1+K) .* (Q3_y - Q2_y);
Q5 = [Q5_x; Q5_y];
% Q5_vx = (1+K) .* Q3_vx;
% Q5_vy = (1+K) .* Q3_vy;
% Q5_v = (1+K) .* Q3_v;


Q5_vy = diff(Q5_y);
Q5_v = diff(Q5);

% 以下为chebychev多出来的那根连杆
Q6_x=Q5_x-L;
Q6_y=Q5_y;
Q6 = [Q6_x; Q6_y];
% Q6_vx = (1+K) .* Q3_vx;
% Q6_vy = (1+K) .* Q3_vy;
% Q6_v = (1+K) .* Q3_v;


Q6_vy = Q5_vy ;
Q6_v = Q5_v;

dis_of_two_point=P6_y - Q6_y;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 以上为另一侧的连杆机构 q %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%%%%%%%%%% 以下开始安装梳状抓手 %%%%%%%%%%%%%%%%%%%%%%%%%%%

comb_start_x=P6_x;
comb_end_x=P6_x-c;

comb_y=zeros(5,iit);


comb_y(1,:)=P6_y-comb_gap2*(3-1);
comb_y(2,:)=P6_y-comb_gap2*(3-2);
comb_y(3,:)=P6_y;
comb_y(4,:)=P6_y-comb_gap*(3-4);
comb_y(5,:)=P6_y-comb_gap*(3-5);


%%%%%%%%%%%%%%%%%%%%%%%%%%% 以上完成安装梳状抓手 %%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%% 以下开始安装另一个梳状抓手 %%%%%%%%%%%%%%%%%%%%%%%%%%%

comb_start_x_Q=Q6_x;
comb_end_x_Q=Q6_x-c;

comb_y_Q=zeros(5,iit);

for comb_Q_N=1:5

comb_y_Q(comb_Q_N,:)=Q6_y-comb_gap*(3-comb_Q_N);     % 梳状抓手共5个step，总长200,第一个step在最下面

comb_Q_N=comb_Q_N+1;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%% 以上完成安装另一个梳状抓手 %%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%%%%%%%%% 以下设置梯子的坐标 %%%%%%%%%%%%%%%%%%%%%%%%%%
gap=300;    % gap 为梯子间距

% 第一个（最下面的）梯子
la1_left=-85;
la1_right=la1_left+38;
la1_low_0=-210+0;
la1_up_0=la1_low_0+25;

% 第二个梯子(在第一个上面)
la2_left=-85;
la2_right=la2_left+38;
la2_low_0=la1_low_0+gap;
la2_up_0=la1_up_0+gap;

% 第三个梯子(在第二个上面)
la3_left=-85;
la3_right=la3_left+38;
la3_low_0=la2_low_0+gap;
la3_up_0=la2_up_0+gap;

% 第四个梯子(在第三个上面)
la4_left=-85;
la4_right=la4_left+38;
la4_low_0=la3_low_0+gap;
la4_up_0=la3_up_0+gap;


% 运动的第二个梯子(在第一个上面)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 以下开始运动 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 以下开始运动 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 以下开始运动 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 第一步 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for ii1=1:iit

% la2_up_0=125 

% 以下寻找comb_y初始位置高于la2_up_0 的最低的那一级step
for comb_N=1:5

comb_selected=find(comb_y>=la2_up_0);

min_comb_start1=min(comb_selected);

end
% 以上找到了


% comb_N1=min_comb_start1;
comb_N1=1;

    if comb_y(comb_N1,ii1) < la2_up_0   
        
la2_up(1,ii1)=comb_y(comb_N1,ii1);
la2_low(1,ii1)=la2_up(1,ii1)-25;

la1_up(1,ii1)=la2_up(1,ii1)-300;
la1_low(1,ii1)=la2_low(1,ii1)-300;

la3_up(1,ii1)=la2_up(1,ii1)+gap;
la3_low(1,ii1)=la2_low(1,ii1)+gap;

la4_up(1,ii1)=la3_up(1,ii1)+gap;
la4_low(1,ii1)=la3_low(1,ii1)+gap;


    else

la4_up(1,ii1)=la4_up_0;
la4_low(1,ii1)=la4_low_0;

la3_up(1,ii1)=la3_up_0;
la3_low(1,ii1)=la3_low_0;

la2_up(1,ii1)=la2_up_0;
la2_low(1,ii1)=la2_low_0;

la1_up(1,ii1)=la1_up_0;
la1_low(1,ii1)=la1_low_0;


    end

%%%%%%%%%%%%%%%%%%%%%%%%%%% 以下记录作用在rung上的时间
    if ii1>1 && ii1<80
        if comb_y(comb_N1,ii1) < la2_up_0 
% 记录作用在ladder rung上的起始时间
combP1_work(1,ii1)=ii1(1);
comb_P1_work=find (combP1_work> 0);
comb_P1_start=min(comb_P1_work);
        else
% 记录作用在ladder rung上的终止时间
combP1_notwork(1,ii1)=ii1(1);
comb_P1_notwork= find (combP1_notwork>0);
comb_P1_end=min(comb_P1_notwork);
        end
    end

    ii1=ii1+1;

end

t_combP_1=(comb_P1_end-comb_P1_start)*delta_t;     % delta_t为间隔时差，0.05s




%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 第二步 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ii_st2=50;
for ii=ii_st2 : iit

% la2_up_0=125 

comb_N1_Q=5;


    if comb_y_Q(comb_N1_Q,ii) < comb_y(1,ii) + 300

       % if comb_y_Q(comb_N1_Q,ii) < comb_y(comb_N1,ii1)


        % 记录作用在ladder rung上的起始时间
        if ii>ii_st2 && ii<150
comb_Q1_on(1,ii)=ii(1);
comb_Q1_work=find (comb_Q1_on> 0);
comb_Q1_start=min(comb_Q1_work);
        end
        
la3_up(1,ii)=comb_y_Q(comb_N1_Q,ii);
la3_low(1,ii)=la3_up(1,ii)-25;

la1_up(1,ii)=la3_up(1,ii)-2*gap;
la1_low(1,ii)=la3_low(1,ii)-2*gap;

la2_up(1,ii)=la3_up(1,ii)-gap;
la2_low(1,ii)=la3_low(1,ii)-gap;

la4_up(1,ii)=la3_up(1,ii)+gap;
la4_low(1,ii)=la3_low(1,ii)+gap;

    else

% 记录结束作用在ladder rung上的时间
         if ii>ii_st2+30 && ii<200
comb_Q1_off(1,ii)=ii(1);
comb_Q1_notwork= find (comb_Q1_off>0);
comb_Q1_end=min(comb_Q1_notwork);
         end


la2_up(1,ii)=comb_y(comb_N1,ii);
la2_low(1,ii)=la2_up(1,ii)-25;

la1_up(1,ii)=la2_up(1,ii)-300;
la1_low(1,ii)=la2_low(1,ii)-300;

la3_up(1,ii)=la2_up(1,ii)+gap;
la3_low(1,ii)=la2_low(1,ii)+gap;

la4_up(1,ii)=la3_up(1,ii)+gap;
la4_low(1,ii)=la3_low(1,ii)+gap;




    end

    ii=ii+1;

end

t_combQ_1=(comb_Q1_end-comb_Q1_start)*delta_t;     % delta_t为间隔时差，0.05s









% 在之后的迭代中，在下降的过程中，如果某个step位于la2_up之下，则la2_up随那个step运动
% 相当于修正之前已经形成的la2_up数组，让la2_up再减小

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 第三步 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 找到ii=90的索引（或者时刻）
ii_st3=92;
for ii=ii_st3:iit

% for comb_N2=1:5
    comb_N2=1;

if comb_y(comb_N2,ii) - comb_y_Q(comb_N1_Q,ii) <= 0


        % 记录作用在ladder rung上的起始时间
        if ii>ii_st3 && ii<200
comb_P2_on(1,ii)=ii(1);
comb_P2_work=find (comb_P2_on> 0);
comb_P2_start=min(comb_P2_work);
        end


la3_up(1,ii)=comb_y(comb_N2,ii);
la3_low=la3_up-25;

la2_up(1,ii)=la3_up(1,ii)-gap;
la2_low(1,ii)=la3_low(1,ii)-gap;

la1_up(1,ii)=la2_up(1,ii)-gap;
la1_low(1,ii)=la2_low(1,ii)-gap;

la4_up(1,ii)=la3_up(1,ii)+gap;
la4_low(1,ii)=la3_low(1,ii)+gap;

else 
la2_up(1,ii)=comb_y_Q(comb_N1_Q,ii);
la2_low(1,ii)=la2_up(1,ii)-25;

la1_up(1,ii)=la2_up(1,ii)-gap;
la1_low(1,ii)=la2_low(1,ii)-gap;

la3_up(1,ii)=la2_up(1,ii)+gap;
la3_low(1,ii)=la2_low(1,ii)+gap;

la4_up(1,ii)=la3_up(1,ii)+gap;
la4_low(1,ii)=la3_low(1,ii)+gap;


% 记录结束作用在ladder rung上的时间
         if ii>ii_st3+30 && ii<300
comb_P2_off(1,ii)=ii(1);
comb_P2_notwork= find (comb_P2_off>0);
comb_P2_end=min(comb_P2_notwork);
        end

end    
ii=ii+1;
% end
end

t_combP_2=(comb_P2_end-comb_P2_start)*delta_t;     % delta_t为间隔时差，0.05s





%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 第四步 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 找到ii=120的索引（或者时刻）
ii_st4=133;
for ii=ii_st4:iit

% for comb_N2=1:5
    comb_N2_Q=5;

if comb_y_Q(comb_N2_Q,ii) <= comb_y(comb_N2,ii) +300


       % 记录作用在ladder rung上的起始时间
        if ii>ii_st4 && ii<250
comb_Q2_on(1,ii)=ii(1);
comb_Q2_work=find (comb_Q2_on> 0);
comb_Q2_start=min(comb_Q2_work);
        end


la4_up(1,ii)=comb_y_Q(comb_N2_Q,ii);
la4_low=la4_up-25;

la3_up(1,ii)=la4_up(1,ii)-gap;
la3_low(1,ii)=la4_low(1,ii)-gap;

la2_up(1,ii)=la3_up(1,ii)-gap;
la2_low(1,ii)=la3_low(1,ii)-gap;

la1_up(1,ii)=la2_up(1,ii)-gap;
la1_low(1,ii)=la2_low(1,ii)-gap;



else 
la4_up(1,ii)=comb_y(comb_N2,ii);
la4_low=la4_up-25;

la3_up(1,ii)=la4_up(1,ii)-gap;
la3_low(1,ii)=la4_low(1,ii)-gap;

la2_up(1,ii)=la3_up(1,ii)-gap;
la2_low(1,ii)=la3_low(1,ii)-gap;

la1_up(1,ii)=la2_up(1,ii)-gap;
la1_low(1,ii)=la2_low(1,ii)-gap;



% % 记录结束作用在ladder rung上的时间
         if ii>ii_st4+30 && ii<300
comb_Q2_off(1,ii)=ii(1);
comb_Q2_notwork= find (comb_Q2_off>0);
comb_Q2_end=min(comb_Q2_notwork);
        end

end    
ii=ii+1;
% end
end

t_combQ_2=(comb_Q2_end-comb_Q2_start)*delta_t;     % delta_t为间隔时差，0.05s




%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 第五步 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%% 抓la3
ii_st5=172;
for ii=ii_st5:iit

    comb_N3=1;

if comb_y(comb_N3,ii) < comb_y_Q(comb_N2_Q,ii)


%       % 记录作用在ladder rung上的起始时间
        if ii>ii_st5 && ii<300
comb_P3_on(1,ii)=ii(1);
comb_P3_work=find (comb_P3_on> 0);
comb_P3_start=min(comb_P3_work);
        end

la4_up(1,ii)=comb_y(comb_N3,ii);
la4_low(1,ii)=la4_up(1,ii)-25;


la3_up(1,ii)=la4_up(1,ii)-gap;
la3_low(1,ii)=la4_low(1,ii)-gap;

la2_up(1,ii)=la3_up(1,ii)-gap;
la2_low(1,ii)=la3_low(1,ii)-gap;

la1_up(1,ii)=la2_up(1,ii)-gap;
la1_low(1,ii)=la2_low(1,ii)-gap;

else 
la3_up(1,ii)=comb_y_Q(comb_N2_Q,ii);
la3_low=la3_up-25;

la2_up(1,ii)=la3_up(1,ii)-gap;
la2_low(1,ii)=la3_low(1,ii)-gap;

la1_up(1,ii)=la2_up(1,ii)-gap;
la1_low(1,ii)=la2_low(1,ii)-gap;

la4_up(1,ii)=la3_up(1,ii)+gap;
la4_low(1,ii)=la3_low(1,ii)+gap;



% % 记录结束作用在ladder rung上的时间
         if ii>ii_st5+30 && ii<300
comb_P3_off(1,ii)=ii(1);
comb_P3_notwork= find (comb_P3_off>0);
comb_P3_end=min(comb_P3_notwork);
        end
 
ii=ii+1;
end
end

t_combP_3=(comb_P3_end-comb_P3_start)*delta_t;     % delta_t为间隔时差，0.05s






%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 第六步 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%% 再抓la3
ii_st6=205;
for ii=ii_st6:iit


    % comb_N4_Q=5;
    comb_N4_Q=1;    % 原本应该是5，但是因为到头来，所以是1


if comb_y_Q(comb_N4_Q,ii) - comb_y(1,ii) <= 0


%      % 记录作用在ladder rung上的起始时间
        if ii>ii_st6 && ii<iit
comb_Q3_on(1,ii)=ii(1);
comb_Q3_work=find (comb_Q3_on> 0);
comb_Q3_start=min(comb_Q3_work);
        end

la4_up(1,ii)=comb_y_Q(comb_N4_Q,ii);
la4_low=la4_up-25;

% la3_up(1,ii)=la4_up(1,ii)-gap;
% la3_low(1,ii)=la4_low(1,ii)-gap;

la3_up(1,ii)=la4_up(1,ii)-gap;
la3_low(1,ii)=la4_low(1,ii)-gap;

la2_up(1,ii)=la3_up(1,ii)-gap;
la2_low(1,ii)=la3_low(1,ii)-gap;

la1_up(1,ii)=la2_up(1,ii)-gap;
la1_low(1,ii)=la2_low(1,ii)-gap;

else 

la4_up(1,ii)=comb_y(comb_N3,ii);
la4_low(1,ii)=la4_up(1,ii)-25;


la3_up(1,ii)=la4_up(1,ii)-gap;
la3_low(1,ii)=la4_low(1,ii)-gap;

la2_up(1,ii)=la3_up(1,ii)-gap;
la2_low(1,ii)=la3_low(1,ii)-gap;

la1_up(1,ii)=la2_up(1,ii)-gap;
la1_low(1,ii)=la2_low(1,ii)-gap;



% % 记录结束作用在ladder rung上的时间
         if ii>ii_st6+30 && ii<iit
comb_Q3_off(1,ii)=ii(1);
comb_Q3_notwork= find (comb_Q3_off>0);
comb_Q3_end=min(comb_Q3_notwork);
        end
 
 
ii=ii+1;
end
end

t_combQ_3=(comb_Q3_end-comb_Q3_start)*delta_t;     % delta_t为间隔时差，0.05s





%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 第七步 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%% 抓la4
ii_st7=250;
for ii=250:iit

    comb_N5=1;

if comb_y(comb_N5,ii) < comb_y_Q(5,ii)


%  % 记录作用在ladder rung上的起始时间
       if ii>ii_st7 && ii<iit
comb_P4_on(1,ii)=ii(1);
comb_P4_work=find (comb_P4_on> 0);
comb_P4_start=min(comb_P4_work);
        end

la4_up(1,ii)=comb_y(comb_N5,ii);
la4_low=la4_up-25;

la3_up(1,ii)=la4_up(1,ii)-gap;
la3_low(1,ii)=la4_low(1,ii)-gap;

la2_up(1,ii)=la3_up(1,ii)-gap;
la2_low(1,ii)=la3_low(1,ii)-gap;

la1_up(1,ii)=la2_up(1,ii)-gap;
la1_low(1,ii)=la2_low(1,ii)-gap;

else 

la4_up(1,ii)=comb_y_Q(comb_N4_Q,ii);
la4_low=la4_up-25;

la3_up(1,ii)=la4_up(1,ii)-gap;
la3_low(1,ii)=la4_low(1,ii)-gap;

la2_up(1,ii)=la3_up(1,ii)-gap;
la2_low(1,ii)=la3_low(1,ii)-gap;

la1_up(1,ii)=la2_up(1,ii)-gap;
la1_low(1,ii)=la2_low(1,ii)-gap;


% % 记录结束作用在ladder rung上的时间
         if ii>ii_st7+30 && ii<iit
comb_P4_off(1,ii)=ii(1);
comb_P4_notwork= find (comb_P4_off>0);
comb_P4_end=min(comb_P4_notwork);
        end
 
ii=ii+1;
end
end

t_combP_4=(comb_P4_end-comb_P4_start)*delta_t;     % delta_t为间隔时差，0.05s





%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 第八步 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%% 抓la4
ii_st8=282;
for ii=ii_st8:iit

    comb_N6_Q=1;

if comb_y_Q(comb_N6_Q,ii) < comb_y(3,ii)


%      % 记录作用在ladder rung上的起始时间
       if ii>ii_st8 && ii<iit
comb_Q4_on(1,ii)=ii(1);
comb_Q4_work=find (comb_Q4_on> 0);
comb_Q4_start=min(comb_Q4_work);
       end


la4_up(1,ii)=comb_y_Q(comb_N6_Q,ii);
la4_low=la4_up-25;

la3_up(1,ii)=la4_up(1,ii)-gap;
la3_low(1,ii)=la4_low(1,ii)-gap;

la2_up(1,ii)=la3_up(1,ii)-gap;
la2_low(1,ii)=la3_low(1,ii)-gap;

la1_up(1,ii)=la2_up(1,ii)-gap;
la1_low(1,ii)=la2_low(1,ii)-gap;

else 

la4_up(1,ii)=comb_y(comb_N5,ii);
la4_low=la4_up-25;

la3_up(1,ii)=la4_up(1,ii)-gap;
la3_low(1,ii)=la4_low(1,ii)-gap;

la2_up(1,ii)=la3_up(1,ii)-gap;
la2_low(1,ii)=la3_low(1,ii)-gap;

la1_up(1,ii)=la2_up(1,ii)-gap;
la1_low(1,ii)=la2_low(1,ii)-gap;


% 记录结束作用在ladder rung上的时间

comb_Q4_end=iit;    % 因为是最后一组了，所以就是最终结束的时间
 
ii=ii+1;
end
end

t_combQ_4=(comb_Q4_end-comb_Q4_start)*delta_t;     % delta_t为间隔时差，0.05s


% 计算机器人的速度，为梯子速度的反速度
la1_up_Vy=diff(la1_up)./diff(t);
robot_Vy=-la1_up_Vy;


% 计算机器人的加速度
robot_a=diff(robot_Vy)./diff(t1);

%计算机器人的攀爬距离
robot_Dr=-la2_up;




% % 以下循环目的在于让第二级梯子下降到最低点后不会随着机器人回升
% for ii=1:iit-1
% 
%     % la2
% if la2_up(ii)<la2_up(ii+1)
%     la2_up(ii+1)=la2_up(ii);
% end
% if la2_low(ii)<la2_low(ii+1)
%     la2_low(ii+1)=la2_low(ii);
% end
% 
% 
% % la1
%     if la1_up(ii)<la1_up(ii+1)
%     la1_up(ii+1)=la1_up(ii);
%     end
% if la1_low(ii)<la1_low(ii+1)
%     la1_low(ii+1)=la1_low(ii);
% end
% 
% 
% % la3
%     if la3_up(ii)<la3_up(ii+1)
%     la3_up(ii+1)=la3_up(ii);
%     end
% if la3_low(ii)<la3_low(ii+1)
%     la3_low(ii+1)=la3_low(ii);
% end
% 
% 
% % la4
%     if la4_up(ii)<la4_up(ii+1)
%     la4_up(ii+1)=la4_up(ii);
%     end
% if la4_low(ii)<la4_low(ii+1)
%     la4_low(ii+1)=la4_low(ii);
% end
% 
% 
% ii=ii+1;
% 
% end


%%%%%%%%%%%%%%%%%%%%%%%%%%% 以上设置梯子的坐标 %%%%%%%%%%%%%%%%%%%%%%%%%%


% 开始统计抓手P的接触时间
for ii=1:iit
% comb_P=[comb_P1_work comb_P2_work comb_P3_work comb_P4_work];

if ii >= comb_P1_start && ii<=comb_P1_end 
comb_P(1,ii)=1;

else if ii >= comb_P2_start && ii<=comb_P2_end 
comb_P(1,ii)=1;

else if ii >= comb_P3_start && ii<=comb_P3_end 
comb_P(1,ii)=1;

else if ii >= comb_P4_start && ii<=comb_P4_end comb_P(1,ii)=1;

else
    comb_P(1,ii)=0;
end
end
end
end

ii=ii+1;
end
% 统计结束


% 开始统计抓手Q的接触时间
for ii=1:iit

if ii >= comb_Q1_start && ii<=comb_Q1_end 
comb_Q(1,ii)=1;

else if ii >= comb_Q2_start && ii<=comb_Q2_end 
comb_Q(1,ii)=1;

else if ii >= comb_Q3_start && ii<=comb_Q3_end 
comb_Q(1,ii)=1;

else if ii >= comb_Q4_start && ii<=comb_Q4_end 
comb_Q(1,ii)=1;

else
    comb_Q(1,ii)=0;
end
end
end
end

ii=ii+1;
end
% 统计结束








for i=1:length(t)

    %%%%%%%%%%%%%%%%%%%%%%%%%% 以下为连杆P %%%%%%%%%%%%%%%%%%%%%%%%%%
    % ani = subplot(2,1,1);
    P1_circle = viscircles(P1',0.05);
    P2_circle = viscircles(P2(:,i)',0.05);
    P3_circle = viscircles(P3(:,i)',0.05);
    P4_circle = viscircles(P4',0.05);
    P5_circle = viscircles(P5(:,i)',0.05);

    A_bar = line([P1(1) P2(1,i)],[P1(2) P2(2,i)],'LineWidth',2,'Color','red');
    B_bar = line([P2(1,i) P3(1,i)],[P2(2,i) P3(2,i)],'LineWidth',2,'Color','red');
    C_bar = line([P3(1,i) P4(1)],[P3(2,i) P4(2)],'LineWidth',2,'Color','red');


    F_bar = line([P3(1,i) P5(1,i)],[P3(2,i) P5(2,i)],'LineWidth',2,'Color','red');      % P5
    L_bar = line([P5(1,i) P6(1,i)],[P5(2,i) P6(2,i)],'LineWidth',2,'Color','red');      % P6


    %%%%%%%%%%%%%%%%%%%%%%%%%%% 以下为连杆Q %%%%%%%%%%%%%%%%%%%%%%%%%%
    % ani = subplot(2,1,1);
    Q1_circle = viscircles(Q1',0.05);
    Q2_circle = viscircles(Q2(:,i)',0.05);
    Q3_circle = viscircles(Q3(:,i)',0.05);
    Q4_circle = viscircles(Q4',0.05);
    Q5_circle = viscircles(Q5(:,i)',0.05);
    
    A_bar_Q = line([Q1(1) Q2(1,i)],[Q1(2) Q2(2,i)],'LineWidth',2,'Color','b');
    B_bar_Q = line([Q2(1,i) Q3(1,i)],[Q2(2,i) Q3(2,i)],'LineWidth',2,'Color','b');
    C_bar_Q = line([Q3(1,i) Q4(1)],[Q3(2,i) Q4(2)],'LineWidth',2,'Color','b');
    
    F_bar_Q = line([Q3(1,i) Q5(1,i)],[Q3(2,i) Q5(2,i)],'LineWidth',2,'Color','b');      % P5
    L_bar_Q = line([Q5(1,i) Q6(1,i)],[Q5(2,i) Q6(2,i)],'LineWidth',2,'Color','b');      % P6



%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 以下勾画梳状抓手 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

comb1= line([comb_start_x(1,i),comb_end_x(1,i)],[comb_y(1,i),comb_y(1,i)],'LineWidth',2,'Color','red');
% comb2= line([comb_start_x(1,i),comb_end_x(1,i)],[comb_y(2,i),comb_y(2,i)],'LineWidth',2);
comb3= line([comb_start_x(1,i),comb_end_x(1,i)],[comb_y(3,i),comb_y(3,i)],'LineWidth',2,'Color','red');
% comb4= line([comb_start_x(1,i),comb_end_x(1,i)],[comb_y(4,i),comb_y(4,i)],'LineWidth',2);
comb5= line([comb_start_x(1,i),comb_end_x(1,i)],[comb_y(5,i),comb_y(5,i)],'LineWidth',2,'Color','red');
comb_bar= line([comb_start_x(1,i),comb_start_x(1,i)],[comb_y(1,i),comb_y(5,i)],'LineWidth',2,'Color','red');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 以上勾画梳状抓手 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 以下勾画另一个梳状抓手 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

comb1_Q= line([comb_start_x_Q(1,i),comb_end_x_Q(1,i)],[comb_y_Q(1,i),comb_y_Q(1,i)],'LineWidth',2,'LineWidth',2,'Color','b');
% comb2_Q= line([comb_start_x_Q(1,i),comb_end_x_Q(1,i)],[comb_y_Q(2,i),comb_y_Q(2,i)],'LineWidth',2);
comb3_Q= line([comb_start_x_Q(1,i),comb_end_x_Q(1,i)],[comb_y_Q(3,i),comb_y_Q(3,i)],'LineWidth',2,'LineWidth',2,'Color','b');
% comb4_Q= line([comb_start_x_Q(1,i),comb_end_x_Q(1,i)],[comb_y_Q(4,i),comb_y_Q(4,i)],'LineWidth',2);
comb5_Q= line([comb_start_x_Q(1,i),comb_end_x_Q(1,i)],[comb_y_Q(5,i),comb_y_Q(5,i)],'LineWidth',2,'LineWidth',2,'Color','b');
comb_bar_Q= line([comb_start_x_Q(1,i),comb_start_x_Q(1,i)],[comb_y_Q(1,i),comb_y_Q(5,i)],'LineWidth',2,'LineWidth',2,'Color','b');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 以上勾画另一个梳状抓手 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    % 插入一条：勾画机器人外轮廓
    R1R2= line([-A,-A],[0,300]);
    R2R3=line([-A,1.1*A],[300,D]);
    R3R4=line([1.1*A,1.05*A],[D,-1.1*A]);
    R4R5=line([1.1*A,-A],[-1.1*A,-300]);
    R5R1=line([-A,-A],[-300,0]);
    % 勾画完成


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 以下插入梯子轮廓 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % 第一个（最下面的）梯子

  line_la1_left = line([la1_left,la1_left],[la1_low(1,i),la1_up(1,i)],'LineWidth',2,'Color','k');
  line_la1_right = line([la1_right,la1_right],[la1_low(1,i),la1_up(1,i)],'LineWidth',2,'Color','k');
  line_la1_up = line([la1_left,la1_right],[la1_up(1,i),la1_up(1,i)],'LineWidth',2,'Color','k');
  line_la1_low = line([la1_left,la1_right],[la1_low(1,i),la1_low(1,i)],'LineWidth',2,'Color','k');

%   la1_x=[la1_left,la1_left,la1_right,la1_right];
%   la1_y=[la1_low,la1_up,la1_up,la1_low];
% 
% patch(la1_x,la1_y,'b'), hold on;    % patch表示填充图形


% % 第二个梯子(在第一个上面)


  line_la2_left = line([la2_left(1),la2_left(1)],[la2_low(1,i),la2_up(1,i)],'LineWidth',2,'Color','k');
  line_la2_right = line([la2_right(1),la2_right(1)],[la2_low(1,i),la2_up(1,i)],'LineWidth',2,'Color','k');
  line_la2_up = line([la2_left(1),la2_right(1)],[la2_up(1,i),la2_up(1,i)],'LineWidth',2,'Color','k');
  line_la2_low = line([la2_left(1),la2_right(1)],[la2_low(1,i),la2_low(1,i)],'LineWidth',2,'Color','k');


% la2_low=la2_low+P6_vy.*t1;
% la2_up=la2_up+P6_vy.*t1;


%   la2_x=[la2_left,la2_left,la2_right,la2_right];
%   la2_y=[la2_low,la2_up,la2_up,la2_low];
% 
% patch(la2_x,la2_y,'b'), hold on;  

% % 第三个梯子(在第二个上面)
  line_la3_left = line([la3_left(1),la3_left(1)],[la3_low(1,i),la3_up(1,i)],'LineWidth',2,'Color','k');
  line_la3_right = line([la3_right(1),la3_right(1)],[la3_low(1,i),la3_up(1,i)],'LineWidth',2,'Color','k');
  line_la3_up = line([la3_left(1),la3_right(1)],[la3_up(1,i),la3_up(1,i)],'LineWidth',2,'Color','k');
  line_la3_low = line([la3_left(1),la3_right(1)],[la3_low(1,i),la3_low(1,i)],'LineWidth',2,'Color','k');


  % % 第四个梯子(在第三个上面)
  line_la4_left = line([la4_left(1),la4_left(1)],[la4_low(1,i),la4_up(1,i)],'LineWidth',2,'Color','k');
  line_la4_right = line([la4_right(1),la4_right(1)],[la4_low(1,i),la4_up(1,i)],'LineWidth',2,'Color','k');
  line_la4_up = line([la4_left(1),la4_right(1)],[la4_up(1,i),la4_up(1,i)],'LineWidth',2,'Color','k');
  line_la4_low = line([la4_left(1),la4_right(1)],[la4_low(1,i),la4_low(1,i)],'LineWidth',2,'Color','k');



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 以上插入梯子轮廓 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    
    % axis(ani, 'equal');
    set(gca,'Xlim',[-200 200],'Ylim',[-400 350],'FontName','Times New Roman','FontSize',20,'LineWidth',1.5);
    
    str1 = '*';
    str2 = '*';

    % P5_text = text(P5(1,i),P5(2,i)+0.4,str1);       % P5
    % P6_text = text(P6(1,i),P6(2,i)+0.4,str2);       % P6

    pause(0.005)
    if i<length(t)

        % 删除连杆P的轨迹
        delete(P1_circle);
        delete(P2_circle);
        delete(P3_circle);
        delete(P4_circle);
        delete(P5_circle);
        delete(A_bar);
        delete(B_bar);
        delete(C_bar);

        delete(F_bar);     % P5
        delete(L_bar);     % P5



        % 删除连杆Q的轨迹
        delete(Q1_circle);
        delete(Q2_circle);
        delete(Q3_circle);
        delete(Q4_circle);
        delete(Q5_circle);
        delete(A_bar_Q);
        delete(B_bar_Q);
        delete(C_bar_Q);
        
        delete(F_bar_Q);     % Q5
        delete(L_bar_Q);     % Q5
        
%         delete(P5_text);      %P5
        % delete(Time);

% 删除梳状抓手轨迹
        delete(comb1)
        % delete(comb2)
        delete(comb3)
        % delete(comb4)
        delete(comb5)
        delete(comb_bar)


% 删除另一个梳状抓手轨迹
        delete(comb1_Q)
        % delete(comb2_Q)
        delete(comb3_Q)
        % delete(comb4_Q)
        delete(comb5_Q)
        delete(comb_bar_Q)



% 删除梯子轨迹
        delete(line_la1_left);
        delete(line_la1_right);
        delete(line_la1_up);
        delete(line_la1_low);


        delete(line_la2_left);
        delete(line_la2_right);
        delete(line_la2_up);
        delete(line_la2_low);


        delete(line_la3_left);
        delete(line_la3_right);
        delete(line_la3_up);
        delete(line_la3_low);


        delete(line_la4_left);
        delete(line_la4_right);
        delete(line_la4_up);
        delete(line_la4_low);


        % vel = subplot(2,1,2);
        % plot(vel,t(1:i),comb_P(1:i),'r'), hold on;        % P
        % plot(vel,t(1:i),comb_Q(1:i),'b'), hold on;        % Q

        % set(vel, 'XLim',[0 T],'YLim',[-1 2]);
        % xlabel(vel, 'Time (s)');
        % ylabel(vel, 'Contact condition');        % contact (1) / uncontact (0)
        % title(vel, 'Condition of each comb mechanism');
        % grid on;
    end
   


end



