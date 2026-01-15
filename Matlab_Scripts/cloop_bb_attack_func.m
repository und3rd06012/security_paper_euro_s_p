function [x,dot_x,err,u,u_amp_lim_state,no_bb_swtc,cum_no_bb_swtc,S,xm]=cloop_bb_attack_func(N,dt,dd,r,dot_r,x,u_amp_lim_up,u_amp_lim_low,add_noise,nn,a_s)

   u_amp_lim_state=zeros(1,N);
   no_bb_swtc=0;
   cum_no_bb_swtc=zeros(1,N);
   xm=zeros(1,N);

   for i=2:1:N

      xm(i-1)=x(i-1)+a_s(i-1);

      err(i-1)=r(i-1)-xm(i-1);

      if (xm(i-1)>r(i-1))
         u(i-1)=u_amp_lim_low;
         u_amp_lim_state(i-1)=-1;
      elseif (xm(i-1)<r(i-1))
         u(i-1)=u_amp_lim_up;
         u_amp_lim_state(i-1)=1;
      else
         if (i>2)
            u(i-1)=u(i-2);
         else
            u(i-1)=0;
         end;
         u_amp_lim_state(i-1)=0;
      end;

      if (i>2)
         if (u(i-1)~=u(i-2))
            no_bb_swtc=no_bb_swtc+1;
         end;
      end;
      cum_no_bb_swtc(i-1)=no_bb_swtc;

      x(i)=dt*(x(i-1)^dd+u(i-1))+x(i-1);

      xm(i)=x(i)+a_s(i);

      dot_x(i-1)=(x(i)-x(i-1))/dt;

      if (add_noise=='y')
         S(i-1)=nn(i-1)/(r(i-1)-err(i-1));
      else
         S(i-1)=0;
      end;

   end;
