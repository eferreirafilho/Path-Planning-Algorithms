function C= rotationtoworldframe(xstart,xgoal)

theta = atan2(xgoal(2)-xstart(2), xgoal(1) - xstart(1));


C=[cos(theta) -sin(theta);sin(theta) cos(theta)];

