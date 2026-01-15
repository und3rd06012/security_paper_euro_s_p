clc;
close all;
clear all;

f_start=1;
f_stop=1000;
f_step=25;

t_start=0;
t_stop=1/f_step;
dt=0.00001;
x(1)=0.1;

ctrl_type=0;

Tr_orbit=10;

u_amp_lim='n';
u_amp_lim_up=70;
u_amp_lim_low=-70;

Tr_const=5;

Kp=500;
Kd=0.2;
Ki=750;

Kd2=0.000002;

Ki2=10;

z=10;
p=30;

amp_db='y';
angle_rad='n';
en_graphs='y';
en_printout='y';

   N=floor((t_stop-t_start)/dt)+1;
   N_f=floor((f_stop-f_start)/f_step)+1;

   ave_ctrl_fft=zeros(1,N-1);
   ave_P_fft=zeros(1,N-1);
   ave_oloop_fft=zeros(1,N-1);
   ave_cloop_fft=zeros(1,N);

   for i=1:1:N_f
      freq(i)=f_start+(i-1)*f_step;
      s_freq=freq(i);
      if (ctrl_type==1)
         u_amp_lim_up=70+6*s_freq;
         u_amp_lim_low=-70-6*s_freq;;
      end;

      [t_info(i,:),BW_sig(i,:),BW_blk(i,:),r_fft,err_fft,u_fft,x_fft]=MControl_Model_v1_2_func(t_start,t_stop,dt,x,ctrl_type,Tr_orbit,Kp,Kd,Kd2,Ki,Ki2,z,p,s_freq,u_amp_lim,u_amp_lim_up,u_amp_lim_low);

      temp_ctrl_fft=u_fft./err_fft;
      temp_P_fft=x_fft(1:N-1)./u_fft;
      temp_oloop_fft=x_fft(1:N-1)./err_fft;
      temp_cloop_fft=x_fft./r_fft;

      ave_ctrl_fft=ave_ctrl_fft+temp_ctrl_fft;
      ave_P_fft=ave_P_fft+temp_P_fft;
      ave_oloop_fft=ave_oloop_fft+temp_oloop_fft;
      ave_cloop_fft=ave_cloop_fft+temp_cloop_fft;

   end;

      ave_ctrl_fft=ave_ctrl_fft/N_f;
      ave_P_fft=ave_P_fft/N_f;
      ave_oloop_fft=ave_oloop_fft/N_f;
      ave_cloop_fft=ave_cloop_fft/N_f;

      ctrl_syst=ifft(ave_ctrl_fft);
      ctrl_fft_amp=abs(ave_ctrl_fft);
      ctrl_fft_angle=angle(ave_ctrl_fft);

      P_syst=ifft(ave_P_fft);
      P_fft_amp=abs(ave_P_fft);
      P_fft_angle=angle(ave_P_fft);

      oloop_syst=ifft(ave_oloop_fft);
      oloop_fft_amp=abs(ave_oloop_fft);
      oloop_fft_angle=angle(ave_oloop_fft);

      cloop_syst=ifft(ave_cloop_fft);
      cloop_fft_amp=abs(ave_cloop_fft);
      cloop_fft_angle=angle(ave_cloop_fft);

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

      figure(fig);
      gcf; shg;
      i=1:1:floor(N/2);
      subplot(2,1,1)
      if (amp_db~='y')
         plot((i-1)/dt/N,ctrl_fft_amp(i))
      else
         semilogx(((i-1)/dt/N),20*log10(ctrl_fft_amp(i)))
      end;
      box on; grid;
      title(' C(s) FFT Amplitude')

      if (amp_db~='y')
         ylabel(' Amplitude ');
      else
         ylabel(' Amplitude (db) ');
      end;
      subplot(2,1,2)
      if (angle_rad~='y')
         if (amp_db~='y')
            plot((i-1)/dt/N,180/pi*ctrl_fft_angle(i))
         else
            semilogx(((i-1)/dt/N),180/pi*ctrl_fft_angle(i))
         end;
      else
         if (amp_db~='y')
            plot((i-1)/dt/N,ctrl_fft_angle(i))
         else
            semilogx(((i-1)/dt/N),ctrl_fft_angle(i))
         end;
      end;
      box on; grid;
      title(' C(s) FFT angle')
      if (amp_db~='y')
         xlabel(' Frequency ');
      else
         xlabel(' Log(Frequency) ');
      end;
      if (angle_rad=='y')
         ylabel(' Phase (rad)');
      else
         ylabel(' Phase (deg)');
      end;
      fig=fig+1;

      figure(fig);
      gcf; shg;
      i=1:1:floor(N/2);
      subplot(2,1,1)
      if (amp_db~='y')
         plot((i-1)/dt/N,P_fft_amp(i))
      else
         semilogx(((i-1)/dt/N),20*log10(P_fft_amp(i)))
      end;
      box on; grid;
      title(' P_c(s) FFT Amplitude')

      if (amp_db~='y')
         ylabel(' Amplitude ');
      else
         ylabel(' Amplitude (db) ');
      end;
      subplot(2,1,2)
      if (angle_rad~='y')
         if (amp_db~='y')
            plot((i-1)/dt/N,180/pi*P_fft_angle(i))
         else
            semilogx(((i-1)/dt/N),180/pi*P_fft_angle(i))
         end;
      else
         if (amp_db~='y')
            plot((i-1)/dt/N,P_fft_angle(i))
         else
            semilogx(((i-1)/dt/N),P_fft_angle(i))
         end;
      end;
      box on; grid;
      title(' P_c(s) FFT angle')
      if (amp_db~='y')
         xlabel(' Frequency ');
      else
         xlabel(' Log(Frequency) ');
      end;
      if (angle_rad=='y')
         ylabel(' Phase (rad)');
      else
         ylabel(' Phase (deg)');
      end;
      fig=fig+1;

      figure(fig);
      gcf; shg;
      i=1:1:floor(N/2);
      subplot(2,1,1)
      if (amp_db~='y')
         plot((i-1)/dt/N,oloop_fft_amp(i))
      else
         semilogx(((i-1)/dt/N),20*log10(oloop_fft_amp(i)))
      end;
      box on; grid;
      title(' L(s) FFT Amplitude')

      if (amp_db~='y')
         ylabel(' Amplitude ');
      else
         ylabel(' Amplitude (db) ');
      end;
      subplot(2,1,2)
      if (angle_rad~='y')
         if (amp_db~='y')
            plot((i-1)/dt/N,180/pi*oloop_fft_angle(i))
         else
            semilogx(((i-1)/dt/N),180/pi*oloop_fft_angle(i))
         end;
      else
         if (amp_db~='y')
            plot((i-1)/dt/N,oloop_fft_angle(i))
         else
            semilogx(((i-1)/dt/N),oloop_fft_angle(i))
         end;
      end;
      box on; grid;
      title(' L(s) FFT angle')
      if (amp_db~='y')
         xlabel(' Frequency ');
      else
         xlabel(' Log(Frequency) ');
      end;
      if (angle_rad=='y')
         ylabel(' Phase (rad)');
      else
         ylabel(' Phase (deg)');
      end;
      fig=fig+1;

      figure(fig);
      gcf; shg;
      i=1:1:floor(N/2);
      subplot(2,1,1)
      if (amp_db~='y')
         plot((i-1)/dt/N,cloop_fft_amp(i))
      else
         semilogx(((i-1)/dt/N),20*log10(cloop_fft_amp(i)))
      end;
      box on; grid;
      title(' H(s) FFT Amplitude')

      if (amp_db~='y')
         ylabel(' Amplitude ');
      else
         ylabel(' Amplitude (db) ');
      end;
      subplot(2,1,2)
      if (angle_rad~='y')
         if (amp_db~='y')
            plot((i-1)/dt/N,180/pi*cloop_fft_angle(i))
         else
            semilogx(((i-1)/dt/N),180/pi*cloop_fft_angle(i))
         end;
      else
         if (amp_db~='y')
            plot((i-1)/dt/N,cloop_fft_angle(i))
         else
            semilogx(((i-1)/dt/N),cloop_fft_angle(i))
         end;
      end;
      box on; grid;
      title(' H(s) FFT angle')
      if (amp_db~='y')
         xlabel(' Frequency ');
      else
         xlabel(' Log(Frequency) ');
      end;
      if (angle_rad=='y')
         ylabel(' Phase (rad)');
      else
         ylabel(' Phase (deg)');
      end;
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
