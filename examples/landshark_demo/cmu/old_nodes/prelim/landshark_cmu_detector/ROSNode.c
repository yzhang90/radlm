
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


#define N_STATES 4
#define N_SENSORS 2
#define N_SIDES 4
#define N_BOUNDS 2*N_SIDES
#define THETA (2*3.1415926/N_SIDES)

#define T 0.1	// Time step

float PRECISION_P[N_SENSORS] = {4, 3};
float PRECISION_V[N_SENSORS] = {1.5, 2};


// Find elementwise minimums in the double array bounds and store them in array b
void ElementwiseMin( float bounds[][N_BOUNDS], float b[])
{
	int i,j;
	for (i=0; i< N_BOUNDS;i++)
	{
		b[i] = bounds[1][i];
		for (j=2;j<N_SENSORS;j++)
			if (bounds[j][i] < b[i])
				b[i] = bounds[j][i];
	};
}

// Given the measurements x, return the maximum possible acceleration of the vehicle 
float MaxAcceleration( float x[])
{
	// I haven't figured out the best C implementation of this function yet,
	// so just return some number
	return 3.5;
}


/* M is the double array of sensor measurements:
   each	M[i] is an array of N_STATES measurements of the ith sensor.

   b is an array of length N_BOUNDS that contains the right-hand of the inequalities A*x <= b that
   all admissible states x must satisfy at current time.	

   All arrays are viewed as column vectors. E.g., M is a N_STATES x N_SENSORS matrix.
*/
int ROSNode( float M[][N_STATES], float b[]) 
{
	int i=0, j=0 ;

	float bounds[N_SENSORS][N_BOUNDS];
	// bounds = B*M + PRECISION, where matrices B and PRECISION are known 
	for (i=0; i<N_SENSORS;i++)
		for (j=0;j<N_SIDES; j++)
		{
			bounds[i][j] = cos(THETA*j)*M[i][0] + sin(THETA*j)*M[i][1] + PRECISION_P[i];
			bounds[i][j+N_SIDES] = cos(THETA*j)*M[i][2] + sin(THETA*j)*M[i][3] + PRECISION_V[i];
		};

	float b_new[N_BOUNDS];
	ElementwiseMin(bounds, b_new);


	for (i=0; i<N_BOUNDS;i++)
		if (b_new[i]<b[i])
			b[i] = b_new[i];

	float min_d = b[0] + b[1] ;
	float d = 0;
	i=2;
	while (i<N_BOUNDS)
	{	
		d = b[i]+b[i+1]; 
		if (d < min_d)
			min_d = d;
		i += 2;
	};

	int check=-1000;		
	if (min_d < 0)
	{
		check = 0;	// An error is detected
		printf("Smallest distance = %f\n", min_d);
		b = NULL;
	} 
	else
	{
		check = 1;	// No error detected
		float a = 0;	// max acceleration
		for ( i=0; i<N_SENSORS; i++)
		{
			a = MaxAcceleration(M[i]);	// Returns maximal possible acceleration when the vehicle is in this state
			for (j=0;j<N_SIDES;j++)
			{
				bounds[i][j] = bounds[i][j] + T*bounds[i][j+N_SIDES] + a*T*T/2 ;
				bounds[i][j+N_SIDES] =bounds[i][j+N_SIDES] + a*T ;
			};
			ElementwiseMin(bounds,b);
		};		
	};

	return check;
}

int main ()
{
	float M[N_SENSORS][N_STATES];

	float x[N_STATES] = {5,4,1,2} ;	// At position (x,y)=(5,4) driving with velocity (v_x,v_y) = (1,2)
	int i,j;
	for (i=0; i<N_SENSORS;i++)
		for (j=0;j<N_STATES;j++)
			M[i][j] = x[j]; // + rand()/RAND_MAX - 0.5;	// True measurement with some random noise in the range [-1/2, 1/2]

	float b[N_BOUNDS];
	for (j=0;j<N_SIDES; j++)
	{
		b[j] = cos(THETA*j)*x[0] + sin(THETA*j)*x[1] + PRECISION_P[0];
		b[j+N_SIDES] = cos(THETA*j)*x[2] + sin(THETA*j)*x[3] + PRECISION_V[0];
	};

	int check = ROSNode(M, b);

	if (check > 0)
		printf("Everything is fine.\n");
	else
		printf("Detected an error!\n");

	return 0;  
}
