#define CCW -1
#define CW 1
#define COLINEAR 0
#define INTERSECT 1
#define NO_INTERSECT -1

#define MAX_SEGMENTS 200

//#define DRAWEDGES

int CheckConnections(struct Seg _refSeg1);
void AddEdge(int __x1, int __y1, int __x2, int __y2);

#pragma region CREATE GRAPH

int k, j = 0;

struct Seg* seg2 = NULL;
struct Seg* seg3 = NULL;

struct Pxy
{
	int x, y;
};

struct vertice
{
	int v_id;
	int v_px;
	int v_py;
	int neighbours[2*MAX_SEGMENTS+3];
	double weight[2*MAX_SEGMENTS+3];
};

struct vertice nodelist[200];
struct vertice graphlist[200]; 
struct vertice _node1;
struct vertice _node2;

float calcostnodes(struct vertice nd1, struct vertice nd2)
{
	double ax = pow(nd1.v_px-nd2.v_px, 2);
	double ay = pow(nd1.v_py-nd2.v_py, 2);
	return sqrt(ax+ay);
}

int nodeid =0;
int movingnodes_id[3];
struct vertice problematiknode;
int ftn =1;
void CreateNodes()
{
	struct Seg* sg = NULL;
	nodeid = 0;
	for(int _n=0; _n<count_seg;_n++)
	{
		sg = GetSegById(_n);
		struct vertice newnode;
		
		newnode.v_id =nodeid;
		newnode.v_px = sg->_x1;
		newnode.v_py = sg->_y1;

		for(int p0=0; p0<=2*count_seg+3;p0++)
		newnode.neighbours[p0] = 0;

		nodelist[newnode.v_id] = newnode;
		//if(_n==0)printf("\nNode- Create Nodes[0]=[%d,%d]",nodelist[_n].v_px, nodelist[_n].v_py);
		if(ftn ==1 && _n==0) problematiknode = newnode;
		ftn =0;
		nodeid++;
		newnode.v_id =nodeid;
		newnode.v_px = sg->_x2;
		newnode.v_py = sg->_y2;
		nodelist[newnode.v_id] = newnode;
		nodeid++;
	}	
	movingnodes_id[0] = nodeid;	/* Robot1 node id. */
	nodeid++;
	movingnodes_id[1] = nodeid;	/* Robot2 node id. */
	nodeid++;	
	movingnodes_id[2] = nodeid;	/* Request point node id. */
}

void CreateConnetions()
{
	struct Seg _refSeg;
	struct vertice tmp_graph[200];
	struct vertice _node, _nodex;
	for(int ad=0; ad<=nodeid; ad++)
	{ 
		if(ad ==0){ nodelist[ad] = problematiknode;}// I couldn't find the reason why this nodelist[0] keeps changing!!! A fast solution.
		_node = nodelist[ad];
		_refSeg._id= 99;
		_refSeg._x1 = _node.v_px; 
		_refSeg._y1 = _node.v_py;

		for(int _av=0; _av<=nodeid;_av++)
		{
			_nodex = nodelist[_av];
			_refSeg._x2 = _nodex.v_px;
			_refSeg._y2 = _nodex.v_py;
			if(CheckConnections(_refSeg)==1)
			{
				_node.neighbours[_av] =1;
				_node.weight[_av] = calcostnodes(_node, _nodex);
				tmp_graph[ad] = _node;
			}
		}
	}
	for(j=0; j<=200; j++)
	graphlist[j] = tmp_graph[j];
}

void AddReqEdges(struct Pxy _reqp)
{
	for(int _k=0; _k<count_seg;_k++)
	{
		seg2 = GetSegById(_k);
		struct Seg _refSeg;
		_refSeg._id=-99;
		_refSeg._x1 = _reqp.x;
		_refSeg._y1 = _reqp.y;
		_refSeg._x2 = seg2->_x1;
		_refSeg._y2 = seg2->_y1;
		CheckConnections(_refSeg);
		_refSeg._x1 = _reqp.x;
		_refSeg._y1 = _reqp.y;
		_refSeg._x2 = seg2->_x2;
		_refSeg._y2 = seg2->_y2;
		CheckConnections(_refSeg);
	}
}

void PrintGraph()
{
	
	int count_alledges =0;
	for(int o=0; o<2*count_seg+3;o++)
	{
		printf("Node[%d] = [%d,%d]\n",graphlist[o].v_id, graphlist[o].v_px, graphlist[o].v_py);
		for(int d=0; d<=2*count_seg+3;d++)
		{ 
		if(graphlist[o].neighbours[d]==1)
			{  
			count_alledges++;
			}
		}
		}
}

#pragma endregion

#pragma region CHECK INTERSECTION
int orientation(struct Pxy p_1, struct Pxy p_2, struct Pxy p_3)
{
	int D_abc = p_1.x*p_2.y + p_2.x*p_3.y + p_3.x*p_1.y - p_2.x*p_1.y - p_3.x*p_2.y - p_1.x*p_3.y;
	if(D_abc == 0)
		return COLINEAR;
	if(D_abc >0)
		return CCW;	
	if(D_abc < 0)
		return CW;
}


int CheckIntersection(struct Seg _seg1, struct Seg _seg2)	/* Check intersection of _seg1 and _seg2. */
{
	struct Pxy p = {_seg1._x1, _seg1._y1}; //segm[0] = [p,q];
	struct Pxy q = {_seg1._x2, _seg1._y2}; 
	struct Pxy r = {_seg2._x1, _seg2._y1}; //segm[1] = [r,s];
	struct Pxy s = {_seg2._x2, _seg2._y2}; 

	int c1 = orientation(p,q,r);
	int c2 = orientation(p,q,s);
	int c12 = c1*c2;
	int c3 = orientation(r,s,p);
	int c4 = orientation(r,s,q);
	int c34 = c3*c4;

	if(c12<0 && c34 <0)
	{ return INTERSECT; }
	else if(c12*c34 != 0)
	{ return NO_INTERSECT; }
	else
	{ return COLINEAR; }
}

//#define DRAWEDGES

int CheckConnections(struct Seg _refSeg1)
{
	j=0;
	for(j = 0; j<count_seg; j++)
	{
		seg3 =  GetSegById(j);
		struct Seg _cmpSeg;
		_cmpSeg._x1 = seg3->_x1;
		_cmpSeg._y1 = seg3->_y1;
		_cmpSeg._x2 = seg3->_x2;
		_cmpSeg._y2 = seg3->_y2;
		int rsl = CheckIntersection(_refSeg1, _cmpSeg);
		if(rsl == INTERSECT )
			return 0;
	}

#ifdef DRAWEDGES
	AddEdge(_refSeg1._x1, _refSeg1._y1, _refSeg1._x2, _refSeg1._y2);	/* For drawing porpuses. */	
#endif
	return 1;	
}

#pragma endregion

#pragma region EDGES
struct Edge
{
	int id;
	int _x1, _x2, _y1, _y2;
	struct Edge *next;
};

int count_edges =0;
struct Edge *currentedge = NULL;

void AddEdge(int __x1, int __y1, int __x2, int __y2)
{
	struct Edge* temp = (struct Edge*)malloc(sizeof(struct Edge)); //Allocate memory;
	temp->id = count_edges;
	temp->_x1 = __x1;
	temp->_y1 = __y1;
	temp->_x2 = __x2;
	temp->_y2 = __y2;
	temp->next = currentedge;
	currentedge = temp;
	count_edges++;
}

struct Edge* GetEdgeById(int __id)
{
	struct Edge* temp = currentedge;
	if(temp == NULL) 
	{ printf("List is empty\n"); }
	else 
	{ while(temp != NULL) 
	  { if(__id == temp->id){ return temp; }
		temp = temp->next; 
	}}
	return temp;
}

void DeleteEdges()
{
	int iter = count_edges;
	for(int i=0; i<iter;i++)
	{
	struct Edge* temp = currentedge;
	if(currentedge!=NULL)
	{
		currentedge = currentedge->next;
		free(currentedge); count_edges--;
	}}
}
#pragma endregion
