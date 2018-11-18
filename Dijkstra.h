#define NOTVISITED 0
#define EXPLORED 1
#define FINISHED 2
#define INF 9999.0

void initDijkstra(int rbid);
void GetNeighbourId(struct vertice current_v);
void GetSequence(int hist[], int _robid);
int getmin();

int finished[200];
double distance[200];
int travelCost;

struct vertice start_node;
struct vertice end_node;
struct vertice current_node;

int numberofneighbours = 0;
int neighbourids[200];
int roadToSuccess_r1[200];
int roadToSuccess_r2[200];

double Dijkstra(int _robotid){   
	//For first robot:
	initDijkstra(_robotid);				   /* Set visited[i] = NOTVISITED and distance[i] = INF. */
	int prev[200];
	for (int op = 0; op < 200; op++)
	{
		prev[op] = -1;
	}

	int _robotnode;
	if(_robotid == 1) _robotnode = movingnodes_id[0];
	else if(_robotid == 2) _robotnode = movingnodes_id[1];
	else printf("\nRobot ID is wrong, re-check it!\n");
		
	start_node = graphlist[_robotnode]; /* Current position of the robot1 as a node on our graph. */
	current_node = start_node;
	
	distance[_robotnode] = 0; 	   /* Distance from start to start is 0. */
	finished[_robotnode] = FINISHED;	   /* Mark current node as finished. */

	end_node = graphlist[movingnodes_id[2]];   /* Current request position as a node on our graph. */
	
	int loopcount = 0;
	while (loopcount <= movingnodes_id[2])			/* When all are checked just return the result. */
	{
		GetNeighbourId(current_node);		   	/* Get ID-s of neighbours. */
		for (int i = 1; i <= numberofneighbours; i++)
		{
			if (finished[neighbourids[i]] != FINISHED)
			{
				double dist = distance[current_node.v_id] + current_node.weight[neighbourids[i]]; //distance[i] = distance[current,i] + distance[current]
				if (distance[neighbourids[i]] > dist)
				{
					distance[neighbourids[i]] = dist;
					prev[neighbourids[i]] = current_node.v_id;
				}
			}
		}
		finished[current_node.v_id] = FINISHED;
		current_node = graphlist[getmin()];

		loopcount++;
	}
	GetSequence(prev, _robotid);

	#ifdef PRINT	
	printf("Start[%d]=[%d,%d]; Goal[%d]=[%d,%d]\n", start_node.v_id, start_node.v_px, start_node.v_py, end_node.v_id, end_node.v_px, end_node.v_py);
	for(int _i = 1; _i <roadToSuccess[0]+1; _i++)
	{
		printf("--> %d", roadToSuccess[_i]);
	}
	printf("\n");
	#endif

	return distance[end_node.v_id];
}

void initDijkstra(int _rob_id)
{
	for (int i = 0; i < 200; i++)
	{
		finished[i] = NOTVISITED;
		if(_rob_id ==1)
		roadToSuccess_r1[i] = -50;
		else if(_rob_id ==2)
		roadToSuccess_r2[i] = -50;
		distance[i] = INF;
	}
}

void GetNeighbourId(struct vertice current_v)
{
	numberofneighbours = 0;
	for (int klm = 0; klm <= movingnodes_id[2]; klm++)
		neighbourids[klm] = 0;

	for (int i = 0; i <= movingnodes_id[2]; i++)
	{
		if (i != current_v.v_id)
			if (current_v.neighbours[i] == 1 && finished[i] != FINISHED)
			{
				numberofneighbours++; /* To lower the number of iteration on main func. */
				neighbourids[numberofneighbours] = i;	/* Just add id-s in order. */
			}
	}
}

int getmin()
{
	double minvalue = INF + 10;
	int nodeid = -1;
	for (int i = 0; i <= movingnodes_id[2]; i++)
	{
		if (minvalue > distance[i] && finished[i] != FINISHED)
		{
			minvalue = distance[i];
			nodeid = i;
		}
	}
	return nodeid;
}

void GetSequence(int hist[], int _rbid)
{
	int s = end_node.v_id;
	if(_rbid ==1 )
	{
		roadToSuccess_r1[1] = end_node.v_id;
		int seqcounter = 2;	/* There's reason for everything. */
		while (1)
		{
			roadToSuccess_r1[seqcounter] = hist[s];
			s = hist[s];
			seqcounter++;
			if (s == start_node.v_id)
			{
				roadToSuccess_r1[seqcounter] = hist[s];
				break;
			}
		}
		roadToSuccess_r1[0] = seqcounter - 1; //On this position we write how many steps we needed.
	}
	else
	{
		roadToSuccess_r2[1] = end_node.v_id;
		int seqcounter = 2;	/* There's reason for everything. */
		while (1)
		{
			roadToSuccess_r2[seqcounter] = hist[s];
			s = hist[s];
			seqcounter++;
			if (s == start_node.v_id)
			{
				roadToSuccess_r2[seqcounter] = hist[s];
				break;
			}
		}
		roadToSuccess_r2[0] = seqcounter - 1; //On this position we write how many steps we needed.
	}

}

