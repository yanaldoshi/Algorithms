/*
PROGRAM TO CALCULATE SHORTEST PATH USING ASTAR ALGORITHM.
GRID NOTATIONS: 
				STANDARD MATRIX NOTATIONS (ROW AND COLUMNS STARTING WITH 0)
GLOBAL VARIABLES:
				(minx,maxx) 		: Row extremities(dimensions)
				(miny,maxy) 		: Column extremities(dimensions)
				path[maxx*maxy][2]	: Potential path array, 1st dimension is nodes, 2nd dimension is node coordinates
				obst				: Length of obstacle array
				obstacles[obst][2] 	: Obstacle array, 1st dimension is array length, 2nd dimension is obstacle coordinates
				(pos_x,pos_y)		: Current position
				(prev_x,prev_y)		: Previous position
				
FUNCTIONS:
				float heuristics(int curr[2],int x,int y) : Function to calculate the heuristics of node to destination.
															Returns float value of Euclidean distance between them.
															curr[2]: Node
															(x,y): Destination
				
				void fill_neighbours(int list[4][2],int current_pos[2],short int null_x,short int null_y,int ind,short int coming_from) :
					Function to fill neighbours of current node.
					list[4][2]: Empty list of 4 potential neighbours; up,down,right,left.
					current_pos[2]: Coordinates of current node
					null_x: Boundary in x direction (rows)
					null_y: Boundary in y direction (columns)
					ind: For index maintaining of list array
					coming_from: For keeping track of previous node direction
					
				void fill_list(int list[4][2],int current_pos[2],int previous_pos[2]): 
					Function to determine boundary towards rows or columns and previous moving direction relative to previous node.
					list[4][2]: Empty list of 4 potential neighbours; up,down,right,left.
					current_pos[2]: Coordinates of current node
					previous_pos[2]: Coordinates of previous node
					
				void Astar(int destination_pos[2],int obstacles[obst][2]):
					Function to calculate path.
					destination_pos[2]: Coordinates of destination
					obstacles[obst][2] 	: Obstacle array, 1st dimension is array length, 2nd dimension is obstacle coordinates
				
ISSUES AND LIMITATIONS:
						Currently only 4 directions are supported: Up Down Right Left
						If obstacles are given such that current position is entrapped in them (cannot backtrack), errored path is given.
						
*/
#include<stdio.h>
#include<math.h>

#define minx 0
#define miny 0
#define maxx 7
#define maxy 7
//length of obstacle array
#define obst 1

//Potential Path Array, contains: x,y
int path[maxx*maxy][2];

//Array of previous obstacles, contains: x,y
//Initialize obstacles array here
int obstacles[obst][2]=
{
    {1,1},
};

//Current position and Previous position

int pos_x=0;
int pos_y=0;
int prev_x=0;
int prev_y=0;


//Calculate Heuristic Value: Euclidean Distance
float heuristic(int curr[],int x,int y){
    int dx=x-curr[0];
    int dy=y-curr[1];
    return sqrt(pow(dx,2)+pow(dy,2));
}

//Filling neighbours: Determine neighbours depending on where you are coming from
void fill_neighbours(int list[4][2],int current_pos[2],short int null_x,short int null_y,int ind,short int coming_from)
{

	//north
    if(!(null_x==1) && !(coming_from==1)){
		list[ind][0]=current_pos[0]-1;
        list[ind++][1]=current_pos[1];
	}

	//west
    if(!(null_y==2) && !(coming_from==2)){
        list[ind][0]=current_pos[0];
        list[ind++][1]=current_pos[1]-1;
    }

	//south
    if(!(null_x==3) && !(coming_from==3)){
        list[ind][0]=current_pos[0]+1;
        list[ind++][1]=current_pos[1];
    }

	//east
    if(!(null_y==4) && !(coming_from==4)){
        list[ind][0]=current_pos[0];
        list[ind++][1]=current_pos[1]+1;
    }

	//fill rest as 254 (unattainable)
	for(ind;ind<4;ind++){
        list[ind][0]=254;
        list[ind][1]=254;
    }
    return;
}

//Filling the list of neighbours: Determine boundary, Determine where you are coming from
void fill_list(int list[4][2],int current_pos[2],int previous_pos[2])
{
	/*
		null notations:
		0->nothing
		1->north
		2->west
		3->south
		4->east
	*/
	short int null_x=0; //null_x: boundary on row
    short int null_y=0; //null_y: boundary on column

	/*
		coming_from notations:
		0->nowhere (home)
		1->north
		2->west
		3->south
		4->east
	*/
	short int coming_from=0;
    int ind=0; //Index maintainer for list array

    //boundaries
    if(current_pos[0]+1 > maxx) null_x=3; //Boundary towards South
    else if(current_pos[1]-1 < minx) null_x=1; //Boundary towards North
    if(current_pos[1]+1 > maxy) null_y=4; //Boundary towards East
    else if(current_pos[1]-1 < miny) null_y=2; //Boundary towards West

	//at home
    if(current_pos[0]==previous_pos[0] && current_pos[1]==previous_pos[1])
	{
		//coming from nowhere = i.e start position
		coming_from=0;
		fill_neighbours(list,current_pos,null_x,null_y,ind,coming_from);
		return;
	}

	//coming from north
    if(current_pos[0]-previous_pos[0] == 1){
		coming_from=1;
		fill_neighbours(list,current_pos,null_x,null_y,ind,coming_from);
		return;
	}

	//coming from west
    if(current_pos[1]-previous_pos[1] == 1){
		coming_from=2;
		fill_neighbours(list,current_pos,null_x,null_y,ind,coming_from);
		return;
	}

	//coming from south
    if(current_pos[0]-previous_pos[0] == -1){
		coming_from=3;
		fill_neighbours(list,current_pos,null_x,null_y,ind,coming_from);
		return;
	}

	 //coming from east
    if(current_pos[1]-previous_pos[1] == -1){

		coming_from=4;
		fill_neighbours(list,current_pos,null_x,null_y,ind,coming_from);
		return;
	}
}

void Astar(int destination_pos[2],int obstacles[obst][2])
{
	int i,path_ind=0,lowheur,j; //path_ind: For maintaining index of path array; lowheur: index of nearest neighbor; i,j: For looping purpose
	int openlist_pos[4][2]; //Nearest neighbors: x,y
	int current_pos[2]; //Current position
	int previous_pos[2]; //Previous position
	current_pos[0]=pos_x;
	current_pos[1]=pos_y;
	previous_pos[0]=prev_x;
	previous_pos[1]=prev_y;
	float heurval[4]; //Heuristic values of neighbors
	while(current_pos[0]!=destination_pos[0] || current_pos[1]!=destination_pos[1]){
		//Fill neighbors list
        fill_list(openlist_pos,current_pos,previous_pos);
		//Fill the heuristic value of obstacles (if any in neighbors) as very high (unattainable)
        for(i=0;i<4;i++){
            for(j=0;j<obst;j++){
                if(openlist_pos[i][0]==obstacles[j][0] && openlist_pos[i][1]==obstacles[j][1]){
                openlist_pos[i][0]=254;
                openlist_pos[i][1]=254;
                heurval[i]=999999.9999;
                }
            }
        }

		//Calculate the heuristic value of rest of the neighbors
        for(i=0;i<4;i++){
            if(openlist_pos[i][0]!=254 && openlist_pos[i][1]!=254)
            heurval[i]=heuristic(openlist_pos[i],destination_pos[0],destination_pos[1]);
        }
		//Checking for nearest neighbor
        lowheur=0;
        for(i=0;i<4;i++){
            if(openlist_pos[i][0]!=254){
                if(heurval[i]<heurval[lowheur]) lowheur=i;
            }
        }

		//Filling the nearest neighbor in path array
        path[path_ind][0]=openlist_pos[lowheur][0];
        path[path_ind++][1]=openlist_pos[lowheur][1];
		//Previous node = Current node
        previous_pos[0]=current_pos[0];
        previous_pos[1]=current_pos[1];
		//Current node = Nearest neighbor
        current_pos[0]=openlist_pos[lowheur][0];
        current_pos[1]=openlist_pos[lowheur][1];
        pos_x=current_pos[0];
        pos_y=current_pos[1];
        prev_x=previous_pos[0];
        prev_y=previous_pos[1];
	}

	//debug
	for(i=0;i<path_ind;i++){
	   printf("%d %d\n",path[i][0],path[i][1]);
    }

}

int main()
{
	int dest[2]={3,3};
	Astar(dest,obstacles);
	return 0;
}
