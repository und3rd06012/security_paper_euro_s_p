function [t,r,dot_r]=orbit_func(Tr_const,Tr_orbit,t_start,t_stop,dt,no_freqs,s_freq)

   N=floor((t_stop-t_start)/dt)+1;

   for i=1:1:N
      t(i)=t_start+(i-1)*dt;

      if (Tr_orbit==-1)

         r(i)=0;

      elseif (Tr_orbit==0)

         if (i<N/2)
            r(i)=0;
         else
            r(i)=Tr_const;
         end;

      elseif (Tr_orbit==1)

         if (i<N/2)
            r(i)=Tr_const*t(i);
         else
            r(i)=r(i-1);
         end;

      elseif (Tr_orbit==2)

         if (i<N/2)
            r(i)=Tr_const*t(i)^2;
         else
            r(i)=r(i-1);
         end;

      elseif (Tr_orbit==3)

         if (i<N/2)
            r(i)=Tr_const/2*sin(2*pi*5*t(i));
         else
            r(i)=r(i-1);
         end;

      elseif (Tr_orbit==4)

         if (i<N/2)
            r(i)=Tr_const/10*exp(2*t(i));
         else
            r(i)=r(i-1);
         end;

      elseif (Tr_orbit==5)

         if (i<N/4)
            r(i)=Tr_const*t(i);
         elseif ((i>N/4) && (i<=N/2))
            r(i)=Tr_const*t(i)^2;
         elseif ((i>N/2) && (i<=3*N/4))
            r(i)=Tr_const/2*sin(2*pi*7*t(i));
         else
            r(i)=Tr_const/10*exp(2*t(i));
         end;

      elseif (Tr_orbit==6)

         dur=1000;
         if (i<=N/2-dur)
            r(i)=0;
         elseif ((i>N/2-dur) && (i<=N/2+dur))
            r(i)=1/300*sin(300*pi*(t(i)-N/2*dt))/pi/(t(i)-N/2*dt);
         else
            r(i)=0;
         end;

      elseif (Tr_orbit==7)

         if (i<=N/10)
            r(i)=0;
         elseif ((i>N/10) && (i<=2*N/10))
            r(i)=Tr_const;
         elseif ((i>2*N/10) && (i<=3*N/10))
            r(i)=-Tr_const;
         elseif ((i>4*N/10) && (i<=5*N/10))
            r(i)=0;
         elseif ((i>6*N/10) && (i<=8*N/10))
            r(i)=Tr_const/2*sin(2*pi*7*t(i));
         else
            r(i)=Tr_const/10*exp(2*t(i));
         end;

      elseif (Tr_orbit==8)

         dur=100;
         if (i<=N/4)
            r(i)=0;
         elseif ((i>N/4) && (i<=N/4+dur))
            r(i)=Tr_const;
         elseif ((i>N/4+dur) && (i<=N/2))
            r(i)=-0;
         elseif ((i>N/2) && (i<=N/2+dur))
            r(i)=-Tr_const;
         elseif ((i>N/2+dur) && (i<=N))
            r(i)=0;
         else
            r(i)=0;
         end;

      elseif (Tr_orbit==9)

         r(i)=0;

         for kk=1:1:no_freqs
            r(i)=r(i)+sin(2*pi*kk*t(i));
         end;
         r(i)=r(i)/no_freqs;

      elseif (Tr_orbit==10)

         r(i)=Tr_const/2*sin(2*pi*s_freq*t(i));

      end;

   end;

   dot_r=zeros(1,N);
   for i=1:1:N-1
      dot_r(i)=(r(i+1)-r(i))/dt;
   end;

