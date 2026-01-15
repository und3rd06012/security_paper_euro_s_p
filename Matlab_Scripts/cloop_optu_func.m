function [x,dot_x,err,u,opt_u,pid_ctrl_sig,u_amp_lim_state,ctrl_state,ctrl_state_on,pid_off_const,S]=cloop_optu_func(Kp,Kd,Ki,N,dt,dd,r,dot_r,x,rem_nonl_plant,u_amp_lim,u_amp_lim_up,u_amp_lim_low,perm_ctrl,perm_state,trip_hyst_en,trip_hyst_d,add_noise,nn)

   ctrl_state=zeros(1,N);
   if (perm_ctrl=='y')
      if (perm_state==1)
         for i=1:1:N
            ctrl_state(i)=1;
         end;
      end;
   end;

   trip_flag=0;
   trip_cnt=0;
   pid_off_const=10;
   u_amp_lim_state=zeros(1,N);

   for i=2:1:N

      err(i-1)=r(i-1)-x(i-1);
      if (i>2)
         d_err(i-1)=(err(i-1)-err(i-2))/dt;
      else
         d_err(i-1)=0;
      end;
      i_err(i-1)=sum(err)*dt;

      pid_ctrl_off_sig(i-1)=(Kp*err(i-1)+Kd*d_err(i-1)+Ki*i_err(i-1))/pid_off_const;
      pid_ctrl_on_sig(i-1)=Kp*err(i-1)+Kd*d_err(i-1)+Ki*i_err(i-1);

      if (ctrl_state(i-1)==0)
         pid_ctrl_sig(i-1)=pid_ctrl_off_sig(i-1);
      else
         pid_ctrl_sig(i-1)=pid_ctrl_on_sig(i-1);
      end;

      opt_u(i-1)=dot_r(i-1)-r(i-1)^dd;
      u(i-1)=opt_u(i-1)+pid_ctrl_sig(i-1);

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

      trip_delay=floor(trip_hyst_d/dt)+1;

      if (trip_hyst_en~='y')
         trip_cnt=0;
      else
         if (trip_flag==0)
            trip_cnt=0;
         elseif (trip_flag==1)
            if (trip_cnt<trip_delay)
               trip_cnt=trip_cnt+1;
            else
               trip_cnt=0;
            end;
         end;
      end;

       if (perm_ctrl~='y')
          if (trip_hyst_en~='y')
             if (x(i)>x(i-1))
                if (x(i)>r(i))
                   ctrl_state(i)=1;
                end;
             elseif (x(i)<x(i-1))
                if (x(i)<r(i))
                   ctrl_state(i)=1;
                end;
             else
                ctrl_state(i)=0;
             end;
             ctrl_state_on=i;
          else
             if (trip_cnt==0)
                if (x(i)>x(i-1))
                   if (x(i)>r(i))
                      ctrl_state(i)=1;
                   end;
                elseif (x(i)<x(i-1))
                   if (x(i)<r(i))
                      ctrl_state(i)=1;
                   end;
                else
                   ctrl_state(i)=0;
                end;
                ctrl_state_on=i;
            else
               ctrl_state(i)=ctrl_state(i-1);
            end;
         end;
      else
         if (perm_state==1)
            ctrl_state(i)=1;
         else
            ctrl_state(i)=0;
         end;
         ctrl_state_on=0;
      end;

      if (ctrl_state(i)~=ctrl_state(i-1))
         trip_flag=1;
      end;

      if (add_noise=='y')
         S(i-1)=nn(i-1)/(r(i-1)-err(i-1));
      else
         S(i-1)=0;
      end;

   end;
