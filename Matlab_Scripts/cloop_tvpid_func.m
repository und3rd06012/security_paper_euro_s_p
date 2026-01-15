function [Kp_tv,Kd_tv,Ki_tv,x,dot_x,err,u,u_amp_lim_state,ctrl_state,ctrl_state_on,pid_off_const,S]=cloop_tvpid_func(Kp,Kd,Ki,N,dt,dd,r,dot_r,x,rem_nonl_plant,u_amp_lim,u_amp_lim_up,u_amp_lim_low,trip_hyst_en,trip_hyst_d,add_noise,nn)

   for i=1:1:N
      ctrl_state(i)=1;
   end;
   ctrl_state_on=1;

   pid_off_const=1;
   u_amp_lim_state=zeros(1,N);

   for i=2:1:N

      err(i-1)=r(i-1)-x(i-1);
      if (i>2)
         d_err(i-1)=(err(i-1)-err(i-2))/dt;
      else
         d_err(i-1)=0;
      end;
      i_err(i-1)=sum(err)*dt;

     trip_delay=floor(trip_hyst_d/dt)+1;

      Kp_tv(i-1)=Kp/2+Kp*abs(err(i-1));
      if (Kp_tv(i-1)>500)
         Kp_tv(i-1)=500;
      end;

      Kd_tv(i-1)=Kd/2+Kd*abs(err(i-1));
      if (Kd_tv(i-1)>1)
         Kd_tv(i-1)=1;
      end;

      Ki_tv(i-1)=Ki/2+Ki*abs(err(i-1));
      if (Ki_tv(i-1)>750)
         Ki_tv(i-1)=750;
      end;

      if (trip_hyst_en=='y')
         if (mod(i,trip_delay)==2)
            Kp_s=Kp_tv(i-1);
            Kd_s=Kd_tv(i-1);
            Ki_s=Ki_tv(i-1);
         else
            Kp_tv(i-1)=Kp_s;
            Kd_tv(i-1)=Kd_s;
            Ki_tv(i-1)=Ki_s;
         end;
      end;

      pid_ctrl_on_sig(i-1)=Kp_tv(i-1)*err(i-1)+Kd_tv(i-1)*d_err(i-1)+Ki_tv(i-1)*i_err(i-1);

      if (rem_nonl_plant=='y')
         u(i-1)=-x(i-1)^(dd)+dot_r(i-1)+pid_ctrl_on_sig(i-1);
      else
         u(i-1)=dot_r(i-1)+pid_ctrl_on_sig(i-1);
      end;

      if (u_amp_lim=='y')
         if (u(i-1)>u_amp_lim_up)
            u(i-1)=u_amp_lim_up;
            u_amp_lim_state(i-1)=1;
         end;
         if (u(i-1)<u_amp_lim_low)
            u(i-1)=u_amp_lim_low;
            u_amp_lim_state(i-1)=-1;
         end;
      end;

      x(i)=dt*(x(i-1)^dd+u(i-1))+x(i-1);

      dot_x(i-1)=(x(i)-x(i-1))/dt;

      if (add_noise=='y')
         S(i-1)=nn(i-1)/(r(i-1)-err(i-1));
      else
         S(i-1)=0;
      end;

   end;
