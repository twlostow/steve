M = dlmread('../servo-sim/resp.csv');

t=M(:,1)
err=M(:,2)

f=20*log(abs(fft(err)))/log(10)

semilogx(f)