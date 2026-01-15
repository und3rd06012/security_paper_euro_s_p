clc;
close all;
clear all;

x(1)=0.1;

ctrl_type=0;

f_start=1;
f_stop=40000;
f_step=100;

u_amp_lim='n';
u_amp_lim_up=70;
u_amp_lim_low=-70;

amp_db='y';
angle_rad='n';
en_graphs='y';
en_printout='y';

   N_f=floor((f_stop-f_start)/f_step)+1;

   for i=1:1:N_f
      freq(i)=f_start+(i-1)*f_step;
      s_freq=freq(i);
      if (ctrl_type==1)
         u_amp_lim_up=70+6*s_freq;
         u_amp_lim_low=-70-6*s_freq;;
      end;

      [t_info(i,:),BW_sig(i,:),BW_blk(i,:)]=MControl_Model_v1_2_func(x,ctrl_type,s_freq,u_amp_lim,u_amp_lim_up,u_amp_lim_low);
   end;

   fig=1;
   if (en_graphs=='y')

      figure(fig);
      gcf; shg;
      i=1:1:N_f;
      plot(freq(i),t_info(i,2))
      xlabel(' Frequency (Hz)  ');
      ylabel(' IAE ');
      box on; grid;
      title(' IAE vs Frequency')
      fig=fig+1;

      figure(fig);
      gcf; shg;
      i=1:1:N_f;
      plot(freq(i),t_info(i,3))
      xlabel(' Frequency (Hz)  ');
      ylabel(' ISE ');
      box on; grid;
      title(' ISE vs Frequency')
      fig=fig+1;

      figure(fig);
      gcf; shg;
      i=1:1:N_f;
      plot(freq(i),t_info(i,4))
      xlabel(' Frequency (Hz)  ');
      ylabel(' Total U Energy ');
      box on; grid;
      title(' Total U Energy vs Frequency')
      fig=fig+1;

      figure(fig);
      gcf; shg;
      i=1:1:N_f;
      plot(freq(i),t_info(i,5))
      xlabel(' Frequency (Hz)  ');
      ylabel(' Avg U Power ');
      box on; grid;
      title(' Avg U Power vs Frequency')
      fig=fig+1;

      figure(fig);
      gcf; shg;
      i=1:1:N_f;
      plot(freq(i),t_info(i,6))
      xlabel(' Frequency (Hz)  ');
      ylabel(' Total min U Energy ');
      box on; grid;
      title(' Total min U Energy vs Frequency')
      fig=fig+1;

      figure(fig);
      gcf; shg;
      i=1:1:N_f;
      plot(freq(i),t_info(i,7))
      xlabel(' Frequency (Hz)  ');
      ylabel(' K_u_I_A_E ');
      box on; grid;
      title(' K_u_I_A_E vs Frequency')
      fig=fig+1;

      figure(fig);
      gcf; shg;
      i=1:1:N_f;
      plot(freq(i),t_info(i,8))
      xlabel(' Frequency (Hz)  ');
      ylabel(' K_u_M_S_E ');
      box on; grid;
      title(' K_u_M_S_E vs Frequency')
      fig=fig+1;

      figure(fig);
      gcf; shg;
      i=1:1:N_f;
      plot(freq(i),BW_sig(i,1))
      xlabel(' Frequency (Hz)  ');
      ylabel(' R(s) Bandwidth ');
      box on; grid;
      title(' R(s) Bandwidth vs Frequency')
      fig=fig+1;

      figure(fig);
      gcf; shg;
      i=1:1:N_f;
      plot(freq(i),BW_sig(i,2))
      xlabel(' Frequency (Hz)  ');
      ylabel(' E(s) Bandwidth ');
      box on; grid;
      title(' E(s) Bandwidth vs Frequency')
      fig=fig+1;

      figure(fig);
      gcf; shg;
      i=1:1:N_f;
      plot(freq(i),BW_sig(i,3))
      xlabel(' Frequency (Hz)  ');
      ylabel(' U(s) Bandwidth ');
      box on; grid;
      title(' U(s) Bandwidth vs Frequency')
      fig=fig+1;

      figure(fig);
      gcf; shg;
      i=1:1:N_f;
      plot(freq(i),BW_sig(i,4))
      xlabel(' Frequency (Hz)  ');
      ylabel(' X(s) Bandwidth ');
      box on; grid;
      title(' X(s) Bandwidth vs Frequency')
      fig=fig+1;

      figure(fig);
      gcf; shg;
      i=1:1:N_f;
      plot(freq(i),BW_blk(i,1))
      xlabel(' Frequency (Hz)  ');
      ylabel(' C(s) Bandwidth ');
      box on; grid;
      title(' C(s) Bandwidth vs Frequency')
      fig=fig+1;

      figure(fig);
      gcf; shg;
      i=1:1:N_f;
      plot(freq(i),BW_blk(i,2))
      xlabel(' Frequency (Hz)  ');
      ylabel(' P(s) Bandwidth ');
      box on; grid;
      title(' P(s) Bandwidth vs Frequency')
      fig=fig+1;

      figure(fig);
      gcf; shg;
      i=1:1:N_f;
      plot(freq(i),BW_blk(i,3))
      xlabel(' Frequency (Hz)  ');
      ylabel(' L(s) Bandwidth ');
      box on; grid;
      title(' L(s) Bandwidth vs Frequency')
      fig=fig+1;

      figure(fig);
      gcf; shg;
      i=1:1:N_f;
      plot(freq(i),BW_blk(i,4))
      xlabel(' Frequency (Hz)  ');
      ylabel(' H(s) Bandwidth ');
      box on; grid;
      title(' H(s) Bandwidth vs Frequency')
      fig=fig+1;

   end;

   if (en_printout=='y')
      clc;
      ave_C_BW=sum(BW_blk(:,1))/N_f;
      ave_P_BW=sum(BW_blk(:,2))/N_f;
      ave_L_BW=sum(BW_blk(:,3))/N_f;
      ave_H_BW=sum(BW_blk(:,4))/N_f;

      sim_tbl=table(ave_C_BW,ave_P_BW,ave_L_BW,ave_H_BW,'VariableNames',{'Avg_C_BW_Hz','Avg_P_BW_Hz','Avg_L_BW_Hz','Avg_H_BW_Hz'});

      norm_C_BW=sum(BW_blk(:,1)'.*freq)/sum(freq);
      norm_P_BW=sum(BW_blk(:,2)'.*freq)/sum(freq);
      norm_L_BW=sum(BW_blk(:,3)'.*freq)/sum(freq);
      norm_H_BW=sum(BW_blk(:,4)'.*freq)/sum(freq);

      sim2_tbl=table(norm_C_BW,norm_P_BW,norm_L_BW,norm_H_BW,'VariableNames',{'Norm_C_BW_Hz','Norm_P_BW_Hz','Norm_L_BW_Hz','Norm_H_BW_Hz'});

      disp('------------------------------------------------')
      disp('		Avg Blocks BW			')
      disp('------------------------------------------------')
      disp(sim_tbl)
      disp('------------------------------------------------')
      disp('		Norm Blocks BW			')
      disp('------------------------------------------------')
      disp(sim2_tbl)

   end;
