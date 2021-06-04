#include<stdio.h>
#include <math.h>

struct point {

	double x, y;

}a,b,c;

void main()
{
	a = {140.574,89.094};
	b = {98.092,131.575};
	c = {140.574,89.589};

	float A = (b.y - a.y);
	float B = -(b.x-a.x);
	float C = -a.x*(b.y - a.y) + a.y*(b.x - a.x);

	printf("c点距离a、b所在直线的距离 = %f\n", (A*c.x + B * c.y + C) / (sqrt(A*A + B * B)));
	printf("a、b之间的距离= %f", sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2)));
}