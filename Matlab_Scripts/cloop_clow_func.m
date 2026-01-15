function [x,dot_x,err,u,u_amp_lim_state,ctrl_state,ctrl_state_on,S]=cloop_clow_func(Kp,z,p,N,dt,dd,r,dot_r,x,rem_nonl_plant,u_amp_lim,u_amp_lim_up,u_amp_lim_low,add_noise,nn)

   for i=1:1:N
      ctrl_state(i)=1;
   end;
   ctrl_state_on=1;

   u_amp_lim_state=zeros(1,N);

   y(1)=0;
   y2(1)=0;

   for i=2:1:N

      err(i-1)=r(i-1)-x(i-1);
      if (i>2)
         d_err(i-1)=(err(i-1)-err(i-2))/dt;
      else
         d_err(i-1)=0;
      end;

      y(i)=(-p*y(i-1)+Kp*d_err(i-1)+Kp*z*err(i-1))*dt+y(i-1);
      y2(i)=y(i-1)*dt+y2(i-1);

      if (rem_nonl_plant=='y')
         u(i-1)=-x(i-1)^(dd)+dot_r(i-1)+y2(i-1);
      else
         u(i-1)=dot_r(i-1)+y2(i-1);
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
