clc;
close all;
clear all;

t_start=0;
t_stop=1;
dt=0.00001;
x(1)=0.1;

ctrl_type=6;

Tr_orbit=3;

no_freqs=100;
s_freq=10;

dd=2;

add_noise='y';
snr_noise=40;
filter_noise='y';
filter_order=50;

u_amp_lim='y';
u_amp_lim_up=150;
u_amp_lim_low=-150;

Tr_const=5;

Kd2=0.0000021;

Ki2=321.3681;

Kd=0.0306;
Ki=421.5556;

Kp=125.3641;
z=5.3716;
p=47.6495;

rem_nonl_plant='y';

trip_hyst_en='y';
trip_hyst_d=0.01;

perm_ctrl='n';
perm_state=1;

amp_db='y';
angle_rad='n';
en_graphs='y';
en_time_graphs='y';
en_fft_graphs='n';
en_printout='y';

   N=floor((t_stop-t_start)/dt)+1;

   [t,r,dot_r]=orbit_func(Tr_const,Tr_orbit,t_start,t_stop,dt,no_freqs,s_freq);

   nn=zeros(1,N);
   if (add_noise=='y')
      rnn=awgn(r,snr_noise);
      nn=rnn-r;

      if (filter_noise=='y')

         for i=1:1:N-1
            r(i)=(rnn(i+1)+rnn(i))/2;
         end;
         r(N)=r(N-1);

         for k=1:1:filter_order
            for i=1:1:N-1
               r(i)=(r(i+1)+r(i))/2;
            end;
            r(N)=r(N-1);
         end;

         r(floor(k/2)+1:1:N)=r(1:1:N-floor(k/2));
         r(1:1:floor(k/2))=r(1:1:floor(k/2))-r(floor(k/2)+1);

      else
         r=rnn;
      end;

      for f=0:1:1/dt/2-1
         H(f+1)=1/2^filter_order*(1+exp(-1i*2*pi*f*dt))^filter_order;
      end;
      H_amp=abs(H);

   end;

   dot_r=zeros(1,N);
   for i=1:1:N-1
      dot_r(i)=(r(i+1)-r(i))/dt;
   end;

   for i=1:1:N
      min_u(i)=dot_r(i)-r(i)^dd;
   end;

   r_fft=fft(r);
   r_fft_amp=abs(r_fft);
   r_fft_angle=angle(r_fft);

   if (ctrl_type==0)

      [x,dot_x,err,u,u_amp_lim_state,ctrl_state,ctrl_state_on,pid_off_const,S]=cloop_pid_func(Kp,Kd,Ki,N,dt,dd,r,dot_r,x,rem_nonl_plant,u_amp_lim,u_amp_lim_up,u_amp_lim_low,perm_ctrl,perm_state,trip_hyst_en,trip_hyst_d,add_noise,nn);

   elseif (ctrl_type==1)

      [x,dot_x,err,u,u_amp_lim_state,no_bb_swtc,cum_no_bb_swtc,S]=cloop_bb_func(N,dt,dd,r,dot_r,x,u_amp_lim_up,u_amp_lim_low,add_noise,nn);

      ctrl_state=zeros(1,N);
      ctrl_state_on=1;

   elseif (ctrl_type==2)

      [x,dot_x,err,u,opt_u,pid_ctrl_sig,u_amp_lim_state,ctrl_state,ctrl_state_on,pid_off_const,S]=cloop_optu_func(Kp,Kd,Ki,N,dt,dd,r,dot_r,x,rem_nonl_plant,u_amp_lim,u_amp_lim_up,u_amp_lim_low,perm_ctrl,perm_state,trip_hyst_en,trip_hyst_d,add_noise,nn);

   elseif (ctrl_type==3)

      [x,dot_x,err,u,u_amp_lim_state,ctrl_state,ctrl_state_on,pid_off_const,S]=cloop_stpid_func(Kp,Kd,Ki,N,dt,dd,r,dot_r,x,rem_nonl_plant,u_amp_lim,u_amp_lim_up,u_amp_lim_low,add_noise,nn);

   elseif (ctrl_type==4)

      [x,dot_x,err,u,u_amp_lim_state,ctrl_state,ctrl_state_on,pid_off_const,S]=cloop_stpi2d2_func(Kp,Kd,Kd2,Ki,Ki2,N,dt,dd,r,dot_r,x,rem_nonl_plant,u_amp_lim,u_amp_lim_up,u_amp_lim_low,add_noise,nn);

   elseif (ctrl_type==5)

      [Kp_tv,Kd_tv,Ki_tv,x,dot_x,err,u,u_amp_lim_state,ctrl_state,ctrl_state_on,pid_off_const,S]=cloop_tvpid_func(Kp,Kd,Ki,N,dt,dd,r,dot_r,x,rem_nonl_plant,u_amp_lim,u_amp_lim_up,u_amp_lim_low,trip_hyst_en,trip_hyst_d,add_noise,nn);

   elseif (ctrl_type==6)

      [x,dot_x,err,u,u_amp_lim_state,ctrl_state,ctrl_state_on,S]=cloop_clow_func(Kp,z,p,N,dt,dd,r,dot_r,x,rem_nonl_plant,u_amp_lim,u_amp_lim_up,u_amp_lim_low,add_noise,nn);
      pid_off_const=1;

   end;

   if ((ctrl_type==0) || (ctrl_type==2) || (ctrl_type==3) || (ctrl_type==4) || (ctrl_type==5) || (ctrl_type==6))
      ctrl_type_valid=1;
   else
      ctrl_type_valid=0;
   end;

   x_fft=fft(x);
   x_fft_amp=abs(x_fft);
   x_fft_angle=angle(x_fft);

   err_fft=fft(err);
   err_fft_amp=abs(err_fft);
   err_fft_angle=angle(err_fft);

   u_fft=fft(u);
   u_fft_amp=abs(u_fft);
   u_fft_angle=angle(u_fft);

   cloop_fft=x_fft./r_fft;
   cloop_syst=ifft(cloop_fft);
   cloop_fft_amp=abs(cloop_fft);
   cloop_fft_angle=angle(cloop_fft);

   ctrl_fft=u_fft./err_fft;
   ctrl_syst=ifft(ctrl_fft);
   ctrl_fft_amp=abs(ctrl_fft);
   ctrl_fft_angle=angle(ctrl_fft);

   P_fft=x_fft(1:N-1)./u_fft;
   P_syst=ifft(P_fft);
   P_fft_amp=abs(P_fft);
   P_fft_angle=angle(P_fft);

   oloop_fft=ctrl_fft.*P_fft;
   oloop_syst=ifft(oloop_fft);
   oloop_fft_amp=abs(oloop_fft);
   oloop_fft_angle=angle(oloop_fft);

   fs=1/dt;

   [x_BW]=bwest_func(x_fft,fs,N);
   [r_BW]=bwest_func(r_fft,fs,N);
   [err_BW]=bwest_func(err_fft,fs,N);
   [u_BW]=bwest_func(u_fft,fs,N);

   [cloop_BW]=bwest_func(cloop_fft,fs,N);
   [oloop_BW]=bwest_func(oloop_fft,fs,N);
   [ctrl_BW]=bwest_func(ctrl_fft,fs,N);
   [P_BW]=bwest_func(P_fft,fs,N);

   fig=1;
   if (en_graphs=='y')

      if (en_time_graphs=='y')

      figure(fig);
      gcf; shg;
      i=1:1:N;
      subplot(3,1,3)
      plot(t(i),u_amp_lim_state(i))
      ylim([-1.2 1.2])
      xlabel(' Time (s) ');
      ylabel(' U Limit State ');
      box on; grid;
      subplot(3,1,2)
      if (ctrl_type_valid==1)
         plot(t(i),ctrl_state(i))
         ylim([0 1.2])

         ylabel(' CTRL State ');
      else
         plot(t(i),cum_no_bb_swtc(i))

         ylabel(' Cumul SWT ');
      end;
      box on; grid;
      subplot(3,1,1)
      plot(t(i),r(i),t(i),x(i),'--')

      ylabel(' Amplitude ');
      legend('Ref Input','System Out')
      box on; grid;
      title(' Reference Input vs System Output')
      fig=fig+1;

      figure(fig);
      gcf; shg;
      i=1:1:N-1;
      subplot(3,1,3)
      plot(t(i),u_amp_lim_state(i))
      ylim([-1.2 1.2])
      xlabel(' Time (s) ');
      ylabel(' U Limit State ');
      box on; grid;
      subplot(3,1,2)
      if (ctrl_type_valid==1)
         plot(t(i),ctrl_state(i))
         ylim([0 1.2])

         ylabel(' CTRL State ');
      else
         plot(t(i),cum_no_bb_swtc(i))

         ylabel(' Cumul SWT ');
      end;
      box on; grid;
      subplot(3,1,1)
      plot(t(i),err(i))

      ylabel(' Amplitude ');
      box on; grid;
      title(' Reference Error')
      fig=fig+1;

      figure(fig);
      gcf; shg;
      i=1:1:N-1;
      subplot(3,1,3)
      plot(t(i),u_amp_lim_state(i))
      ylim([-1.2 1.2])
      xlabel(' Time (s) ');
      ylabel(' U Limit State ');
      box on; grid;
      subplot(3,1,2)
      if (ctrl_type_valid==1)
         plot(t(i),ctrl_state(i))
         ylim([0 1.2])

         ylabel(' CTRL State ');
      else
         plot(t(i),cum_no_bb_swtc(i))

         ylabel(' Cumul SWT ');
      end;
      box on; grid;
      subplot(3,1,1)
      plot(t(i),u(i),t(i),min_u(i),'--')

      ylabel(' Amplitude ');
      box on; grid;
      legend('u(t)','min u(t)')
      title(' Control Signal u(i)')
      fig=fig+1;

      figure(fig);
      gcf; shg;
      i=1:1:N;
      subplot(5,1,5)
      plot(t(i),u_amp_lim_state(i))
      ylim([-1.2 1.2])
      xlabel(' Time (s) ');
      ylabel(' U Limit State ');
      box on; grid;
      subplot(5,1,4)
      if (ctrl_type_valid==1)
         plot(t(i),ctrl_state(i))
         ylim([0 1.2])

         ylabel(' CTRL State ');
      else
         plot(t(i),cum_no_bb_swtc(i))

         ylabel(' Cumul SWT ');
      end;
      box on; grid;
      i=1:1:N-1;
      subplot(5,1,3)
      plot(t(i),err(i))

      ylabel(' e(t) ');
      box on; grid;
      subplot(5,1,2)
      plot(t(i),u(i),t(i),min_u(i),'--')

      ylabel(' Amplitude ');
      box on; grid;
      legend('u(t)','min u(t)')
      box on; grid;
      subplot(5,1,1)
      plot(t(i),r(i),t(i),x(i),'--')

      ylabel(' r(t) - x(t) ');
      legend('Ref Input','System Out')
      box on; grid;
      title(' Closed System Signals')
      fig=fig+1;

      if (ctrl_type==2)
         figure(fig);
         gcf; shg;
         i=1:1:N-1;
         plot(t(i),u(i),t(i),opt_u(i),t(i),pid_ctrl_sig(i))
         xlabel(' Time (s) ');
         ylabel(' Amplitude ');
         box on; grid;
         legend('Total u','Opt u','PID u');
         title(' Control u(t) Decomposition')
         fig=fig+1;
      end;

      figure(fig);
      gcf; shg;
      i=1:1:N-1;
      subplot(3,1,3)
      plot(t(i),u_amp_lim_state(i))
      ylim([-1.2 1.2])
      xlabel(' Time (s) ');
      ylabel(' U Limit State ');
      box on; grid;
      subplot(3,1,2)
      if (ctrl_type_valid==1)
         plot(t(i),ctrl_state(i))
         ylim([0 1.2])

         ylabel(' CTRL State ');
      else
         plot(t(i),cum_no_bb_swtc(i))

         ylabel(' Cumul SWT ');
      end;
      box on; grid;
      subplot(3,1,1)
      plot(t(i),abs(dot_r(i).*err(i).*u(i)))

      ylabel(' Amplitude ');
      box on; grid;
      title(' dr/dt x Err x u')
      fig=fig+1;

      if (ctrl_type==5)
         figure(fig);
         gcf; shg;
         i=1:1:N-1;
         plot(t(i),Kp_tv(i))
         xlabel(' Time (s) ');
         ylabel(' Gain ');
         box on; grid;
         title(' Kp Gain')
         fig=fig+1;

         figure(fig);
         gcf; shg;
         i=1:1:N-1;
         plot(t(i),Kd_tv(i))
         xlabel(' Time (s) ');
         ylabel(' Gain ');
         box on; grid;
         title(' Kd Gain')
         fig=fig+1;

         figure(fig);
         gcf; shg;
         i=1:1:N-1;
         plot(t(i),Ki_tv(i))
         xlabel(' Time (s) ');
         ylabel(' Gain ');
         box on; grid;
         title(' Ki Gain')
         fig=fig+1;

      end;

      end;

      figure(fig);
      gcf; shg;
      i=1:1:N-1;
      plot(x(i),dot_x(i))
      xlabel(' x ');
      ylabel(' dx/dt ');
      box on; grid;
      title(' Phase Plane')
      fig=fig+1;

      if (en_fft_graphs=='y')

      figure(fig);
      gcf; shg;
      i=1:1:floor(N/2);
      subplot(2,1,1)
      if (amp_db~='y')
         plot((i-1)/dt/N,r_fft_amp(i))
      else
         semilogx(((i-1)/dt/N),20*log10(r_fft_amp(i)))
      end;
      box on; grid;
      title(' R(s) FFT Amplitude')

      if (amp_db~='y')
         ylabel(' Amplitude ');
      else
         ylabel(' Amplitude (db) ');
      end;
      subplot(2,1,2)
      if (amp_db~='y')
         plot((i-1)/dt/N,x_fft_amp(i))
      else
         semilogx(((i-1)/dt/N),20*log10(x_fft_amp(i)))
      end;
      box on; grid;
      title(' X(s) FFT Amplitude')
      if (amp_db~='y')
         xlabel(' Frequency ');
         ylabel(' Amplitude ');
      else
         xlabel(' Log(Frequency) ');
         ylabel(' Amplitude (db) ');
      end;
      fig=fig+1;

      figure(fig);
      gcf; shg;
      i=1:1:floor(N/2);
      subplot(2,1,1)
      if (amp_db~='y')
         plot((i-1)/dt/N,r_fft_amp(i))
      else
         semilogx(((i-1)/dt/N),20*log10(r_fft_amp(i)))
      end;
      box on; grid;
      title(' R(s) FFT Amplitude')

      if (amp_db~='y')
         ylabel(' Amplitude ');
      else
         ylabel(' Amplitude (db) ');
      end;
      subplot(2,1,2)
      if (angle_rad~='y')
         if (amp_db~='y')
            plot((i-1)/dt/N,180/pi*r_fft_angle(i))
         else
            semilogx(((i-1)/dt/N),180/pi*r_fft_angle(i))
         end;
      else
         if (amp_db~='y')
            plot((i-1)/dt/N,r_fft_angle(i))
         else
            semilogx(((i-1)/dt/N),r_fft_angle(i))
         end;
      end;
      box on; grid;
      title(' R(s) FFT angle')
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
         plot((i-1)/dt/N,err_fft_amp(i))
      else
         semilogx(((i-1)/dt/N),20*log10(err_fft_amp(i)))
      end;
      box on; grid;
      title(' E(s) FFT Amplitude')

      if (amp_db~='y')
         ylabel(' Amplitude ');
      else
         ylabel(' Amplitude (db) ');
      end;
      subplot(2,1,2)
      if (angle_rad~='y')
         if (amp_db~='y')
            plot((i-1)/dt/N,180/pi*err_fft_angle(i))
         else
            semilogx(((i-1)/dt/N),180/pi*err_fft_angle(i))
         end;
      else
         if (amp_db~='y')
            plot((i-1)/dt/N,err_fft_angle(i))
         else
            semilogx(((i-1)/dt/N),err_fft_angle(i))
         end;
      end;
      box on; grid;
      title(' E(s) FFT angle')
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
         plot((i-1)/dt/N,u_fft_amp(i))
      else
         semilogx(((i-1)/dt/N),20*log10(u_fft_amp(i)))
      end;
      box on; grid;
      title(' U(s) FFT Amplitude')

      if (amp_db~='y')
         ylabel(' Amplitude ');
      else
         ylabel(' Amplitude (db) ');
      end;
      subplot(2,1,2)
      if (angle_rad~='y')
         if (amp_db~='y')
            plot((i-1)/dt/N,180/pi*u_fft_angle(i))
         else
            semilogx(((i-1)/dt/N),180/pi*u_fft_angle(i))
         end;
      else
         if (amp_db~='y')
            plot((i-1)/dt/N,u_fft_angle(i))
         else
            semilogx(((i-1)/dt/N),u_fft_angle(i))
         end;
      end;
      box on; grid;
      title(' U(s) FFT angle')
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
         plot((i-1)/dt/N,x_fft_amp(i))
      else
         semilogx(((i-1)/dt/N),20*log10(x_fft_amp(i)))
      end;
      box on; grid;
      title(' X(s) FFT Amplitude')

      if (amp_db~='y')
         ylabel(' Amplitude ');
      else
         ylabel(' Amplitude (db) ');
      end;
      subplot(2,1,2)
      if (angle_rad~='y')
         if (amp_db~='y')
            plot((i-1)/dt/N,180/pi*x_fft_angle(i))
         else
            semilogx(((i-1)/dt/N),180/pi*x_fft_angle(i))
         end;
      else
         if (amp_db~='y')
            plot((i-1)/dt/N,x_fft_angle(i))
         else
            semilogx(((i-1)/dt/N),x_fft_angle(i))
         end;
      end;
      box on; grid;
      title(' X(s) FFT angle')
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

      if (add_noise=='y')
         figure(fig);
         gcf; shg;
         i=1:1:N-1;
         subplot(3,1,3)
         plot(t(i),u_amp_lim_state(i))
         ylim([-1.2 1.2])
         xlabel(' Time (s) ');
         ylabel(' U Limit State ');
         box on; grid;
         subplot(3,1,2)
         if (ctrl_type_valid==1)
            plot(t(i),ctrl_state(i))
            ylim([0 1.2])

            ylabel(' CTRL State ');
         else
            plot(t(i),cum_no_bb_swtc(i))

            ylabel(' Cumul SWT ');
         end;
         box on; grid;
         subplot(3,1,1)
         plot(t(i),S(i))

         ylabel(' S Level ');
         box on; grid;
         title(' Sensitivity Function ')
         fig=fig+1;
      end;

      if (en_time_graphs=='y')

      figure(fig);
      gcf; shg;
      i=1:1:N;
      if (add_noise=='y')
         plot(t(i),r(i),t(i),rnn(i))
      else
         plot(t(i),r(i))
      end;
      xlabel(' Time (s) ');
      ylabel(' Amplitude ');
      ylim([-0.1+min(r) 0.1+max(r)])
      box on; grid;
      title(' Tracking Orbits')
      if (add_noise=='y')
         legend('Noisy Orbit','Filtered Orbit');
      end;
      fig=fig+1;

      if (add_noise=='y')
         figure(fig);
         gcf; shg;
         i=1:1:N;
         plot(t(i),nn(i))
         xlabel(' Time (s) ');
         ylabel(' Amplitude ');
         box on; grid;
         title(' AWGN Noise')
         fig=fig+1;

         if (filter_noise=='y')
            figure(fig);
            gcf; shg;
            f=1:1:1/dt/2;
            if (amp_db~='y')
               plot(f,H_amp(f))
            else
               semilogx(f,20*log10(H_amp(f)))
            end;
            xlabel(' Frequency (Hz) ');
            if (amp_db~='y')
               ylabel(' Amplitude ');
            else
               ylabel(' Amplitude (db)');
            end;
            box on; grid;
            title(' Filter Transfer Function |H(f)|')
            fig=fig+1;
         end;
      end;
      end;

   end;

      tot_err=sum(abs(err))*dt;
      tot_err_ise=(sum(err.^2)*dt)^0.5;
      tot_u_en=sum(u.^2)*dt;
      tot_min_u_en=sum(min_u.^2)*dt;

      ave_u_p=tot_u_en/N;
      fin_err=abs(err(end));
      K_uiae=sum(abs(err.*u))*dt/(t_stop-t_start);
      K_uise=(sum((err.*u).^2)*dt)^0.5/(t_stop-t_start);
      ctrl_util=100*sum(ctrl_state)/N;
      lim_util=100*sum(abs(u_amp_lim_state))/N;

      sim1_tbl=table(Tr_orbit,ctrl_state_on,'VariableNames',{'Tracking_Orbit','CTRL_State_ON'});
      if (ctrl_type_valid==1)
         sim2_tbl=table(Kp,Kd,Ki,'VariableNames',{'PID_Kp','PID_Kd','PID_Ki'});
      end;
      sim3_tbl=table(fin_err,tot_err,tot_err_ise,tot_u_en,ave_u_p,'VariableNames',{'Exit_Err','IAE_Err','ise_Err','U_Energy','U_Power_SimP'});
      if (ctrl_type_valid==1)
         sim4_tbl=table(tot_min_u_en,K_uiae,K_uise,ctrl_util,lim_util,'VariableNames',{'Min_U_Energy','K_uiae','K_uise','ON_State_100','U_Lim_State_100'});
      else
         bb_swtc=100*no_bb_swtc/N;
         sim4_tbl=table(tot_min_u_en,K_uiae,K_uise,bb_swtc,lim_util,'VariableNames',{'Min_U_Energy','K_uiae','K_uise','Switch_100','U_Lim_State_100'});
      end;
      sim5_tbl=table(r_BW,err_BW,u_BW,x_BW,'VariableNames',{'R_BW_Hz','E_BW_Hz','U_BW_Hz','X_BW_Hz'});
      sim6_tbl=table(ctrl_BW,P_BW,oloop_BW,cloop_BW,'VariableNames',{'C_BW_Hz','P_BW_Hz','L_BW_Hz','H_BW_Hz'});

   if (en_printout=='y')

      disp('------------------------------------------------')
      disp('		Control Type				')
      disp('------------------------------------------------')
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
      elseif (ctrl_type==6)
         disp(' 6 - C(s)=K(s+z)/s(s+p) Control')
      end;

      if (ctrl_type==0)
      disp('------------------------------------------------')
      disp('		PID ON Settings			')
      disp('------------------------------------------------')
      disp(sim2_tbl)
      end;

      disp('------------------------------------------------')
      disp('		Performance Metrics		')
      disp('------------------------------------------------')
      disp(sim3_tbl)
      disp(sim4_tbl)
      disp('------------------------------------------------')
      disp('		WB Signals - Blocks		')
      disp('------------------------------------------------')
      disp(sim5_tbl)
      disp(sim6_tbl)

   end;
