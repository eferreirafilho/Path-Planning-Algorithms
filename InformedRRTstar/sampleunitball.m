%Function that Gives a rand number x and y taht is inside a unit 
%circle centered in the origin

function xball=sampleunitball


x=rand*sign(randn);
y=rand*sign(randn);

while norm([x y])>=1
    x=rand*sign(randn);
    y=rand*sign(randn);
end
xball=[x;y];


