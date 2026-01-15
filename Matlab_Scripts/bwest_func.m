function [BW]=bwest_func(fft_vec,fs,N)

   fft_amp=abs(fft_vec);

   sum_f2_amp2=0;
   sum_amp2=0;
   for i=1:1:floor(N/2)
      sum_f2_amp2=sum_f2_amp2+(fs/N*(i-1)*fft_amp(i))^2;
      sum_amp2=sum_amp2+fft_amp(i)^2;
   end;
   sum_f2_amp2=sum_f2_amp2*fs/N;
   sum_amp2=sum_amp2*fs/N;

   BW=(sum_f2_amp2/sum_amp2)^0.5;

