function [x,dot_x,err,u,u_amp_lim_state,ctrl_state,ctrl_state_on,pid_off_const,S]=cloop_stpi2d2_func(Kp,Kd,Kd2,Ki,Ki2,N,dt,dd,r,dot_r,x,rem_nonl_plant,u_amp_lim,u_amp_lim_up,u_amp_lim_low,add_noise,nn)

   for i=1:1:N
      ctrl_state(i)=1;
   end;
   ctrl_state_on=1;

   pid_off_const=2;
   u_amp_lim_state=zeros(1,N);

   for i=2:1:N

      err(i-1)=r(i-1)-x(i-1);
      if (i>2)
         d_err(i-1)=(err(i-1)-err(i-2))/dt;
      else
         d_err(i-1)=0;
      end;
      if (i>3)
         d2_err(i-1)=(d_err(i-1)-d_err(i-2))/dt;
      else
         d2_err(i-1)=0;
      end;
      i_err(i-1)=sum(err)*dt;
      i2_err(i-1)=sum(i_err)*dt;

      pid_ctrl_on_sig(i-1)=Kp*err(i-1)+Kd*d_err(i-1)+Kd2*d2_err(i-1)+Ki*i_err(i-1)+Ki2*i2_err(i-1);

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
