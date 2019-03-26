clear;
close all;

fprintf('Simple pick-and-place application.\n=============================\n\n');

s=Scorbot(Scorbot.MODESIM)
s.home;

teach=1;
if (~teach)
	load('demo.mat');
else
a=s.pendant();
b=s.pendant();
c=s.pendant();
d=s.pendant();
e=s.pendant();
f=s.pendant();
g=s.pendant();
h=s.pendant();


end

fprintf('Press any key to start picking-and-placing.\n');
pause;

miSpeed = 80;
s.changeSpeed(miSpeed);

for i=1:5

s.move(a,1);
s.move(b,1);
s.move(c,1);
s.move(d,1);
s.move(e,1);
s.move(f,1);
s.move(g,1);
s.move(h,1);

end
filename = 'testdata.xlsx';

for i=1:15
pos=s.currentPos();
xlswrite(filename,pos,1,'E1:I5')
disp(pos)

end

s.home();
fprintf('Press a key to delete Scorbot object.\n');
pause;
clear s;
