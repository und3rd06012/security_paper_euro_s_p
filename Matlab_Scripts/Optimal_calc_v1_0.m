clc;
close all;
clear all;

t_start=0;
t_stop=1;
dt=0.00001;
x(1)=0.1;

ctrl_type=1;

Tr_orbit=3;

s_freq=10;

Tr_const=5;

Kp_val=200;
Kd_val=0.05;
Kd2_val=0.000002;
Ki_val=300;
Ki2_val=300;
z_val=10;
p_val=30;

u_amp_lim='n';
u_amp_lim_up_val=70;
u_amp_lim_low_val=-70;

iter=2000;

dd=2;

   opt_u=10^10;
   for i=1:1:iter
      Kp=Kp_val+(rand-0.5)*150;
      Kd=Kd_val+(rand-0.5)*0.05;
      Kd2=Kd2_val+(rand-0.5)*0.0000005;
      Ki=Ki_val+(rand-0.5)*250;
      Ki2=Ki2_val+(rand-0.5)*50;
      z=z_val+(rand-0.5)*10;
      p=p_val+(rand-0.5)*50;
      u_amp_lim_up=u_amp_lim_up_val+(rand-0.5)*50;
      u_amp_lim_low=u_amp_lim_low_val+(rand-0.5)*50;

      [t_info,BW_sig,BW_blk,r_fft,err_fft,u_fft,x_fft]=MControl_Model_v1_2_func(t_start,t_stop,dt,x,ctrl_type,Tr_orbit,Kp,Kd,Kd2,Ki,Ki2,z,p,s_freq,u_amp_lim,u_amp_lim_up,u_amp_lim_low);
      if (opt_u>t_info(4))

         opt(1)=Kp;
         opt(2)=Kd;
         opt(3)=Kd2;
         opt(4)=Ki;
         opt(5)=Ki2;

         opt(6)=z;
         opt(7)=p;

         opt(8)=u_amp_lim_up;
         opt(9)=u_amp_lim_low;

         opt(10)=t_info(1);
         opt(11)=t_info(2);
         opt(12)=t_info(3);
         opt(13)=t_info(4);
         opt(14)=t_info(5);
         opt(15)=t_info(7);
         opt(16)=t_info(8);
         opt(17)=t_info(9);
         opt(18)=t_info(10);

         opt_u=t_info(4);
      end;
      if (mod(i,100)==0)
         clc;
         disp(' Progress (%) ')
         disp(100*i/iter)
      end;
   end;

   clc;

   disp('--------------------------------------------------');
   disp('		Control Type				')
   disp('--------------------------------------------------');
      if (ctrl_type==0)
         disp(' 0 - Switching PID Control')
      elseif (ctrl_type==1)
         disp(' 1 - Bang Bang Control')
      elseif (ctrl_type==2)
         disp(' 2 - Opt min U energy Control')
      elseif (ctrl_type==3)
         disp(' 3 - Static PID Control')
      elseif (ctrl_type==4)
         disp(' 4 - Static PI2D2 Control')
      elseif (ctrl_type==5)
         disp(' 5 - Time Varying PID Control')
      elseif (ctrl_type==5)
         disp(' 6 - C(s)=K(s+z)/s(s+p) Control')
      end;

   if (ctrl_type~=1)
   disp('--------------------------------------------------');
   disp('		Optimal Gains				');
   disp('--------------------------------------------------');
   disp(opt(1:5))
   end;

   if (ctrl_type==6)
   disp('--------------------------------------------------');
   disp('		Optimal Zeros/Poles		');
   disp('--------------------------------------------------');
   disp(opt(6:7))
   end;

   if (ctrl_type==1)
   disp('--------------------------------------------------');
   disp('		Optimal U Limits			');
   disp('--------------------------------------------------');
   disp(opt(8:9))
   end;

   disp('--------------------------------------------------');
   disp('	Fin_Err - IAE - ISE - U_En - U_P	');
   disp('--------------------------------------------------');
   disp(opt(10:14))
   disp('--------------------------------------------------');
   disp('	KuIAE - KuISE - CTRL_U - Lim_U	');
   disp('--------------------------------------------------');
   disp(opt(15:18))

Tr_orbit=3;
t_start=0;
t_stop=0.25;
dt=0.00001;
x(1)=0.1;

