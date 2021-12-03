
function BsplineEx

clc; close all; clear all;


x = linspace(0,1,100);
figure(1)
[N00, N10, N20, N30] = pieceWiseNi0(x);
plot(x,N00,'o',x,N10,'o',x,N20,'o',x,N30,'o')
legend('N00','N10', 'N20', 'N30')
grid on
% hold on

figure(2)
[N01, N11, N21] = pieceWiseNi1(x);
plot(x,N01,'o',x,N11,'o',x,N21,'o')
grid on
legend('N01','N11', 'N21')


figure(3)
[N02, N12] = pieceWiseNi2(x);
plot(x,N02,'o',x,N12,'o')
grid on
legend('N02','N12')

function [N00, N10, N20, N30] = pieceWiseNi0(in)
  s = zeros (size (in));
  N00 = zeros (size (in));
  N10 = zeros (size (in));
  N20 = zeros (size (in));
  N30 = zeros (size (in));
  N00 = in >= 0.00 & in < 0.25;
  N10 = in >= 0.25 & in < 0.50;
  N20 = in >= 0.50 & in < 0.75;
  N30 = in >= 0.75 & in <= 1.00;
  

  
end

function [N01, N11, N21] = pieceWiseNi1(in)
  s = zeros (size (in));
  N01 = zeros (size (in));
  N11 = zeros (size (in));
  N21 = zeros (size (in));
  X01 = in >= 0.00 & in < 0.25;
  X12 = in >= 0.25 & in < 0.50;
  X23 = in >= 0.50 & in < 0.75;
  X34 = in >= 0.75 & in <= 1.00;

  N01=4*in.*X01 + 2*(1-2*in).*X12;
  N11 =  (4*in-1).*X12 + (3-4*in).*X23;
  N21 = 2*(2*in-1).*X23 + 4*(1-in).*X34;
  
end

function [N02, N12] = pieceWiseNi2(in)
  s = zeros (size (in));
  N01 = zeros (size (in));
  N11 = zeros (size (in));
  X01 = in >= 0.00 & in < 0.25;
  X12 = in >= 0.25 & in < 0.50;
  X23 = in >= 0.50 & in < 0.75;
  X34 = in >= 0.75 & in <= 1.00;

  N02=8*in.*in.*X01 + (-1.5+12*in-16.*in.*in).*X12 + (4.5-12*in+8.*in.*in).*X23;
  N12 =  (0.5-4*in+8.*in.*in).*X12 + (-5.5+20*in-16.*in.*in).*X23 + 8*((1-in).^2).*X34;
  
end
end