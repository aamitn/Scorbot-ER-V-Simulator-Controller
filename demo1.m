clear;
close all;

fprintf('Simple pick-and-place application.\n=============================\n\n');

s=Scorbot(Scorbot.MODESIM)
s.home;

teach=1;
if (~teach)
	load('demo.mat');
else
	fprintf('----> Teach the robot where is the location for picking items and press Enter to finish.\n\n');
	suministro=s.pendant();
	fprintf('----> Teach the robot where is the location for approximating to the previous one.\n\n');
	apsuministro=s.pendant();
	fprintf('----> Teach the robot where is the location for placing items.\n\n');
	torre=s.pendant();
	fprintf('----> Teach the robot where is the location for approximating to the previous one.\n\n');
	aptorre=s.pendant();
end

fprintf('Press any key to start picking-and-placing.\n');
pause;

miSpeed = 80;
s.changeSpeed(miSpeed);


s.move(torre,1);


s.home();
fprintf('Press a key to delete Scorbot object.\n');
pause;
clear s;