#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>

#define RANGE_MIN 0
#define RANGE_MAX 100

int generate_problem(int passengers, char *filename);
int gFloors;  
int gAreaSize;
int gFastElevators; 
int gSlowElevators; 
int gFastCapacity;  
int gSlowCapacity;  

char *itoa(int i, char* s, int dummy_radix) {
    sprintf(s, "%d", i);
    return s;
}


int main(int argc, char **argv)
{
	srand( (unsigned)time( NULL ) );
	if (argc != 12) {
		printf("Use the following parameters:\n");
		printf("MinPassengers, MaxPassenegrs, Step, MinID, MaxID, Floors, Area, NumFast, NumSlow, FastCap, SlowCap \n\nwhere:\n\n");
		printf("\tMinPassengers : the minimum number of passengers per problem\n");
		printf("\tMaxPassengers : the minimum number of passengers per problem\n");
		printf("\t         Step : the step to go from MinPassengers to MaxPassengers\n");
		printf("\t        MinID : the start number of number problems of the same size\n");
		printf("\t        MaxID : the end number of number problems of the same size\n");
		printf("\t       Floors : the number of floors for the problem\n");
		printf("\t         Area : the area size (floors covered by of elevators)\n");
		printf("\t      NumFast : the number of fast elevators\n");
		printf("\t      NumSlow : the number of slow elevators\n");
		printf("\t      FastCap : the capacity of fast elevators\n");
		printf("\t      SlowCap : the capacity of slow elevators\n");
		return -1;
	}
	int MinPassengers,MaxPassengers,Step,MinID,MaxID;
	int counter;
	MinPassengers=atoi(argv[1]);
	MaxPassengers=atoi(argv[2]);
	Step=atoi(argv[3]);
	MinID=atoi(argv[4]);
	MaxID=atoi(argv[5]);
	gFloors=atoi(argv[6]);
	gAreaSize=atoi(argv[7]);
	gFastElevators=atoi(argv[8]);
	gSlowElevators=atoi(argv[9]);
	gFastCapacity=atoi(argv[10]);
	gSlowCapacity=atoi(argv[11]);


	int passengers;
	char filename[100];
	for (passengers=MinPassengers;passengers<=MaxPassengers;passengers+=Step)
		for(counter=MinID;counter<=MaxID;counter++)
		{
			char temps[35];
			filename[0]='\0';
			strcat(filename, "p");
			itoa(gFloors,temps,10);
			strcat(filename,temps);
			strcat(filename,"_");
			itoa(passengers,temps,10);
			strcat(filename,temps);
			strcat(filename,"_");
			itoa(counter,temps,10);
			strcat(filename,temps);
			strcat(filename,".txt");
			generate_problem(passengers,filename);
		}
	return 1;
}

int generate_problem(int passengers, char *filename)
{
	FILE *outfile;
	int i,j;
	int r;

	outfile=fopen(filename,"w");
	fprintf(outfile,"%d %d %d %d %d %d %d\n",gFloors, gAreaSize, gFastElevators, gSlowElevators, gFastCapacity, gSlowCapacity, passengers);

	for(i=0;i<gFastElevators; i++)
	{
		r = ((double) rand() / (double) RAND_MAX) * (2*gFloors/gAreaSize+1);
		fprintf(outfile, "%d ", r*gAreaSize/2);
	}
	fprintf(outfile, "\n");

	for(i=0;i<gFloors;i+=gAreaSize)
	{
		for(j=0;j<gSlowElevators;j++)
		{
			r = ((double) rand() / (double) RAND_MAX) * (gAreaSize+1);
			fprintf(outfile, "%d ", i+r);
		}
		fprintf(outfile,"\n");
	}
	
	for(i=0;i<passengers;i++)
	{
		int r1, r2;
		r1 = ((double) rand() / (double) RAND_MAX) * (gFloors+1);
		r2=r1;
		while (r2==r1)
			r2 = ((double) rand() / (double) RAND_MAX) * (gFloors+1);
		fprintf(outfile, "%d %d\n",r1,r2);
	}

	fclose(outfile);
	printf( "Wrote problem description to file:%s\n", filename);
	return 1;
}
