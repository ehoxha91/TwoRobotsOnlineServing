/*	
   gcc -o wnd TwoRobots_001.c -lX11 -lm -L/usr/X11R6/lib	--compile line	
   
   Student:    Ejup Hoxha 
   Semester:   Fall 2018 - 
   Class:      Advanced Algorithms - Dr. Peter Brass
   University: City University of New York - Groove School of Engineering

*/

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xos.h>
#include <X11/Xatom.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "HelperFile.h"			/* Few linked lists used primarly for drawings and logging. */
#include "GraphGenerator001.h"		/* Manipulations with graph. */
#include "Dijkstra.h"			/* Dijkstra algorithm for this problem. */

//#define DEBUG				/* Forget that this egzists*/
#define DRAWNODES			/* If we want to draw nodes of our graph. */
//#define PRINT				/* This one is not needed neither. */
//#define SIMULATEDGRAPH		/* If you don't have a text file ready.*/
//#define ZOOM 1.2			/* Good idea but not implemented. */

#define DisstanceDifference -0.9	/* If we carea about difference in total traveled distance we make this one lower. */
#define DistanceWeight 0.3		/* If we care about traveled distance of the robot we increase this one*/
#define DistanceHighBorder 300		/* Distance high border and low border are borders and if we stay within these borders*/
#define DistanceLowBorder 10		/* distance is weighted with positive value, othervise is used as contra-weight. */

#define XBORDER 700
#define YBORDER 485

Display *display_ptr;
Screen *screen_ptr;
int screen_num;
char *display_name = NULL;
unsigned int display_width, display_height;

Window win;
int border_width;
unsigned int win_width, win_height;
int win_x, win_y;

XWMHints *wm_hints;
XClassHint *class_hints;
XSizeHints *size_hints;
XTextProperty win_name, icon_name;
char *win_name_string = "Two Robots - Online, K=2 Server Problem";
char *icon_name_string = "Icon For Window";

KeySym keyR;				/* a dealie-bob to handle KeyPress Events */	
char pb_txt[255];			/* a char buffer for KeyPress Events */

XEvent report;

/* ################################### Colors ############################################################# */
GC gc, red, green, white, blue, blue_ghost, orange, orange_shadow, black;
GC orange_2, red_3, blue_2, green_3, white_3, wallcolor, yellow;

XGCValues gc_values, gc_orange_v, gc_red_v, gc_blue_v, gc_green_v, gc_white_v, wallcolor_v, gc_yellow_v,
	  gc_orange_v2, gc_blue_v2, gc_green_v3, gc_white_v3, wallcolor_v3, blue_sv, orange_sv, gc_black_v;

Colormap color_map;
XColor tmp_color1, tmp_color2;
unsigned long valuemask = 0;
/*#########################################################################################################*/

/* Robot Algorithm Functions */
#define ROBOT_HEIGHT 12
#define ROBOT_WIDTH 12
#define R1_s_x 10
#define R1_s_y 10
#define R2_s_x 600
#define R2_s_y 450

struct Robot robot1 = {1, R1_s_x, R1_s_y, ROBOT_HEIGHT, ROBOT_WIDTH, 0.0, NULL};
struct Robot robot2 = {2, R2_s_x, R2_s_y, ROBOT_HEIGHT, ROBOT_WIDTH, 0.0, NULL};

void Robot_work(struct Pxy _reqpointxy);	/* Decision maker. Algorithm which decides which robot should move. */
double AssumeMovement(int _rmovedid);		/* Check our situation in the future if we move robot with id=_rmovedid. */
void AddRobotNode();				/* Adds two nodes in the nodelist.*/
void AddReqNode(struct Pxy _pxy); 		/* Add request point as a node in the existing graph. */
void UpdateMovingNodes(struct Pxy _pxy);	/* When we call UpdateGraph, we also call this function to update moving nodes pos. */
void CreateGraph();				/* First time we create the graph. */
void UpdateGraph(struct Pxy _pxy);		/* We have to update position of robot1, robot2 and request point each time. */
void initVar();		
void Move(int);					/* Move the robot to the request point. */
void move_n_draw(int);				/* Draw and calculate after we called Move(int). */
void drawRobotText(int rid);			/* Update text for info. */
void DrawDetails();				/* Draw some more details. */
void about_info();				/* Re/Draw info about our program. */

/* Drawing Functions */
void GetColors();	
void Re_Draw();
int drw_rp = 0;
void draw_line(GC _gc, int _x1, int _y1, int _x2, int _y2);
void draw_segment(GC _gc, int __id, int _isEdge);
void draw_edge(int _x1, int _y1, int _x2, int _y2);
void draw_request(GC _color, int _posx, int _posy, int _savetolist);
void draw_nodes(int _posx, int _posy);
void draw_robot(GC _color, int _posx, int _posy, int _height, int _width);
void drawstring(GC _scolor, int sposx, int sposy, char *text);
void drawint(GC _scolor, int sposx, int sposy, int);
void drawdouble(GC _scolor, int sposx, int sposy, double inttodraw);
void ClearArea(int _psx, int _psy, int _wclear, int _hclear, int _riseExposeEvent); 

#ifdef SIMULATEDGRAPH
void SimulateReadingFile();
#endif

void text_to_segment(char * _buf, int seg_id); /* Convert text file to segment function. */

int main(int argc, char **argv)
{
	FILE *ptr_file;
	char *buf = malloc(200*20);
	ptr_file = fopen(argv[1], "r"); 
	if(!ptr_file)
		return 1;
	int _sg_id = 0;
	while(fgets(buf, 1000, ptr_file)!=NULL)
	{
	    text_to_segment(buf, _sg_id);	/* Get each line and convert it to a segment */
	    _sg_id++;
	}

	/* Create the graph. Create an arbitrary node which we will use as request. */
	struct Pxy arbitraryreq= {robot1.r_px, robot1.r_py};
 	CreateGraph(arbitraryreq);

	/* Open Display: Try to connect to X server. */
	display_ptr = XOpenDisplay(display_name);
	if(display_ptr == NULL)
	{ 
	  printf("Could not open the window"); exit(-1);}
	  printf("Connected to X server %s\n", XDisplayName(display_name));
	  screen_num = DefaultScreen(display_ptr);
      	  screen_ptr = DefaultScreenOfDisplay(display_ptr);
	  color_map = XDefaultColormap(display_ptr, screen_num);
	  display_width = DisplayWidth(display_ptr, screen_num);
	  display_height = DisplayHeight(display_ptr, screen_num);

	/* Create the window. */
	border_width = 10;
	win_x = 0;
	win_y = 0;
	win_width = 900;
	win_height = 500;
	
	win = XCreateSimpleWindow(display_ptr, RootWindow(display_ptr, screen_num),
				  win_x, win_y, win_width, win_height, border_width,
				  BlackPixel(display_ptr, screen_num),
				  BlackPixel(display_ptr, screen_num));
	
	size_hints = XAllocSizeHints();
	wm_hints = XAllocWMHints();
	class_hints = XAllocClassHint();
	
	if(size_hints == NULL || wm_hints == NULL || class_hints == NULL)
	{
		printf("Error allocating memory for hints.\n"); exit(-1);
	}	

	size_hints -> flags = PPosition | PSize | PMinSize  ;
  	size_hints -> min_width = 60;
  	size_hints -> min_height = 60;
	
	XStringListToTextProperty( &win_name_string,1,&win_name);
	XStringListToTextProperty( &icon_name_string,1,&icon_name);

	wm_hints -> flags = StateHint | InputHint ;
	wm_hints -> initial_state = NormalState;
	wm_hints -> input = False;	
	
	class_hints -> res_name = "x_use_example";
 	class_hints -> res_class = "examples";

	XSetWMProperties( display_ptr, win, &win_name, &icon_name, argv, argc,
                          size_hints, wm_hints, class_hints );	

	/* what events do we want to receive */
	XSelectInput( display_ptr, win, ExposureMask | StructureNotifyMask | ButtonPressMask | KeyPressMask);

	/* Display window on the screen. */
	XMapWindow( display_ptr, win );

	XFlush(display_ptr);

	/* To be able to draw on this window we need to create graphics context. */
	GetColors();	

	while(1)
	{ 
	      XNextEvent(display_ptr, &report );
	      switch( report.type )
	      {
			case Expose:
				Re_Draw(); 
				about_info();
				break;

			case ConfigureNotify:
				win_width = report.xconfigure.width;
				win_height = report.xconfigure.height;
				break;
			case ButtonPress:
			{  
				struct Pxy _pxy = {report.xbutton.x, report.xbutton.y}; 
				if (report.xbutton.button == Button1 )
				{	
					if(_pxy.x < XBORDER && _pxy.y < YBORDER )
					{
						draw_request(green, _pxy.x, _pxy.y, 0);	/* Draw request point. */
						drw_rp =1;
						Robot_work(_pxy);
						Re_Draw();
					}
				}
				else exit(-1);
			}
				break;
			case KeyPress:
			{	
				XLookupString(&report.xkey,pb_txt,255,&keyR,0);
				if(pb_txt[0] == 'e') /* Expose Simulation. */
				{
					ClearArea(0, 0, win_width, win_height, 1);
				}
			}
				break;		
			default: 
			break;
		  }

	}
	exit(0);
	return 0;
}

/* ********************************* */
/* Graph Generator/Helper functions. */
/* ********************************* */

void CreateGraph(struct Pxy _pxy)
{
	CreateNodes();
	AddRobotNode(); 		/* Add/Update robot nodes as moving nodes. */
	AddReqNode(_pxy);		/* Add/Update node of request point. */
	CreateConnetions();		/* Create Graph. */
	
	#ifdef PRINT
	PrintGraph();
	#endif
}

void UpdateGraph(struct Pxy _pxy)
{
	UpdateMovingNodes(_pxy);
	CreateConnetions();
	
	#ifdef PRINT
	PrintGraph();
	#endif
}

void AddRobotNode()			
{
	/* Node of robot 1. */
	struct vertice newnode;
	newnode.v_id = movingnodes_id[0];
	newnode.v_px = robot1.r_px;
	newnode.v_py = robot1.r_py;
	nodelist[newnode.v_id] = newnode;

	/* Node of robot 2. */
	newnode.v_id = movingnodes_id[1];
	newnode.v_px = robot2.r_px;
	newnode.v_py = robot2.r_py;
	nodelist[newnode.v_id] = newnode;
}

void AddReqNode(struct Pxy _pxy)
{
	struct vertice newnode;
	newnode.v_id = movingnodes_id[2];
	newnode.v_px = _pxy.x;
	newnode.v_py = _pxy.y;
	nodelist[newnode.v_id] = newnode;
}

void UpdateMovingNodes(struct Pxy _pxy)
{
	nodelist[movingnodes_id[2]].v_px = _pxy.x;
	nodelist[movingnodes_id[2]].v_py = _pxy.y;
	nodelist[movingnodes_id[1]].v_px = robot2.r_px;
	nodelist[movingnodes_id[1]].v_py = robot2.r_py;
	nodelist[movingnodes_id[0]].v_px = robot1.r_px;
	nodelist[movingnodes_id[0]].v_py = robot1.r_py;
}

#pragma region Algorithm Functions

double lastCost[2] = {0,0};
double cost_robot[2] = {0,0}; 		/* Calculated cost from robot to requested point. */
double distanceDifference = 0; 		/* Traveled distance difference dR = |dR1-dR2|. */
float distance_r1_r2;	 		/* Distance between robot 1 and 2. */
struct Pxy _nextrequest; 		/* Prediction of next request point. MSE?? */
int rts_robot1[200], rts_robot2[200];	/* So we can draw the road to success after we decide which robot to move. */


void move_n_draw(int robid)
{
	GC _col;
	if(robid ==1) _col = blue_2;
	else _col = orange_2;
	struct vertice ver1, ver2;

	if(robid==1)
	{
		int id1 =roadToSuccess_r1[1];
		ver2 = graphlist[id1];
		
		struct Pxy nposxy = {ver2.v_px, ver2.v_py};
		
		#ifdef PRINT
		printf("\n[");
		#endif

		for (int a = 2; a < roadToSuccess_r1[0]+1;a++)
		{
			int id2 = roadToSuccess_r1[a];

			#ifdef PRINT
			printf(" - %d", id2);
			#endif

			ver1 = graphlist[id2];
			
			draw_line(_col, ver2.v_px, ver2.v_py, ver1.v_px, ver1.v_py);
			AddRoad(robid, ver2.v_px, ver2.v_py, ver1.v_px, ver1.v_py);	
			
			ver2 = ver1;
		}
		robot1.r_px = nposxy.x;
		robot1.r_py = nposxy.y;
		//draw_robot(blue, robot1.r_px, robot1.r_py, robot1.r_height, robot1.r_width);
		
		robot1.distance_trv += cost_robot[0];
		lastCost[0] = cost_robot[0];


		#ifdef PRINT
		printf("] \nActual Robot's Position: [%d,%d]\n", nposxy.x, nposxy.y);
		#endif
	}
	else if(robid==2)
	{
		int id1 =roadToSuccess_r2[1];
		ver2 = graphlist[id1];
		
		struct Pxy nposxy = {ver2.v_px, ver2.v_py};	//New Robot position.
		
		#ifdef PRINT
		printf("\n[");
		#endif

		for (int a = 2; a < roadToSuccess_r2[0]+1;a++)
		{
			int id2 = roadToSuccess_r2[a];

			#ifdef PRINT
			printf(" - %d", id2);
			#endif

			ver1 = graphlist[id2];
			
			draw_line(_col, ver2.v_px, ver2.v_py, ver1.v_px, ver1.v_py);
			AddRoad(robid, ver2.v_px, ver2.v_py, ver1.v_px, ver1.v_py);	
			
			ver2 = ver1;
		}

		robot2.r_px = nposxy.x; 
		robot2.r_py = nposxy.y;
		/* We don't need to draw it now, we decided to use ReDraw(). */
		//draw_robot(orange, robot2.r_px, robot2.r_py, robot2.r_height, robot2.r_width);			/* Draw actual position of robot 2. */
		
		robot2.distance_trv += cost_robot[1];
		lastCost[1] = cost_robot[1];
		
		#ifdef PRINT
		printf("] \nActual Robot's Position: [%d,%d]\n", nposxy.x, nposxy.y);
		#endif
	}
}

void Robot_work(struct Pxy _reqpointxy)
{
	UpdateGraph(_reqpointxy);
	initVar();
	cost_robot[0] = Dijkstra(1);
	cost_robot[1] = Dijkstra(2);
	distance_r1_r2 = calcostnodes(graphlist[movingnodes_id[0]], graphlist[movingnodes_id[1]]);
	double movement1 = AssumeMovement(1); 	/* Weight movement of robot1. */
	double movement2 = AssumeMovement(2); 	/* Weight movement of robot2. */
	if(movement1 >movement2)		/* Choose the best possible. */
	Move(1);
	else
	Move(2);
	
	DrawDetails();
}

double disdiffWeight = DisstanceDifference; 	/* Total traveled distance difference weight. */
double distWeight = DistanceWeight;
double simdistHighBorder = DistanceHighBorder;	/* If we will have a larger/smaller distance between robots, */
double simdisLowBorder = DistanceLowBorder;	/* then change the sign of weight, contribute the countrary. */

double AssumeMovement(int _rmovedid)
{
	double weightofmovement = 0.0;  	/* We will give a weight for each movement. Simple step neuron. */
	double sim_td_diff = 0.0; 	    	/* Difference in total traveled distance if we assume to do one movement. */
	double simdistance = 0.0;
	switch (_rmovedid)
	{
		case 1:
			simdistance = cost_robot[1];
			sim_td_diff = robot1.distance_trv + cost_robot[0] - robot2.distance_trv;
			if(sim_td_diff < 0) sim_td_diff *= (-1);
			break;
		case 2:
			simdistance = cost_robot[0];
			sim_td_diff = robot2.distance_trv + cost_robot[1] - robot1.distance_trv;
			if(sim_td_diff < 0) sim_td_diff *= (-1);
			break;
		default:
			break;
	}
	if(simdistance > simdistHighBorder)
	simdistance *=(-1);
	else if(simdistance < simdisLowBorder)
	simdistance *=(-1);

	weightofmovement = disdiffWeight* sim_td_diff + distWeight*simdistance;
	return weightofmovement;
}

void Move(int _robotid)
{
	switch (_robotid)
	{
		case 1:
			move_n_draw(1);
			break;
		case 2:
			move_n_draw(2);
		default:
			break;
	}
}

void drawRobotText(int rid)
{
	switch (rid)
	{
		case 1:
			drawdouble(blue, 825, 15,robot1.distance_trv);
			drawdouble(blue, 825, 45,lastCost[0]);
		break;
		
		case 2:
			drawdouble(orange, 825, 30, robot2.distance_trv);
			drawdouble(orange, 825, 60,lastCost[1]);
		break;
	
		default:
			break;
	}
}

void DrawDetails()
{
	ClearArea(824, 3, 70, 74, 0);
	drawRobotText(1);
	drawRobotText(2);
	distanceDifference = robot1.distance_trv - robot2.distance_trv;
	if(distanceDifference <0)
	distanceDifference *=(-1.0);
	ClearArea(825,75,50,5,0);
	drawdouble(green, 825, 75, distanceDifference);
}

void initVar()
{
	for(int li = 0; li < 200; li++){ rts_robot1[li] = -30; rts_robot2[li] = -30; }
}

#pragma endregion


/* ****************************************** */
/* Drawing functions of segments, robots etc. */
/* ****************************************** */

#pragma region Re/Drawing functions
void Re_Draw()
{ 
	DrawDetails();
	/* Sometimes we may want to draw edges of the possible roads */
	#ifdef DRAWEDGES
	 for(int l55 =0; l55<count_edges;l55++) 
		draw_segment(white, l55,1);
	#endif
	
	/* Re-draw obstacles(segments) read from file. */
	for(int l1 =0; l1<count_seg;l1++)
         draw_segment(wallcolor, l1,0);

	/* Re-draw request points */
	if(drw_rp ==1)
	{ 
	  for(int l3 = 1; l3<=c_r_road;l3++)
	  {
		struct RobotRoad* t_road = GetRoad(l3);
		GC col_;
		
		int hr, wr =0;
		if(t_road->rob_id == 1) {  col_ = blue_2; hr = robot1.r_height; wr =robot1.r_width; }
		else { col_ = orange_2;  hr = robot2.r_height; wr =robot2.r_width;}
		
		if(t_road != NULL)
		{  
		   draw_line(col_, t_road->rx1, t_road->ry1, t_road->rx2, t_road->ry2); 
		   //draw_robot(col_, t_road->rx1, t_road->ry1, hr, wr);
		}
	  }
	  for(int l2 = 0; l2<req_id; l2++)
	  { struct req_point* tmp_rq = GetPoint(l2); draw_request(green,tmp_rq->_req_x, tmp_rq->_req_y, 1);}
	}

	/* Starting position of robots. */
	draw_robot(blue, R1_s_x, R1_s_y, ROBOT_HEIGHT, ROBOT_WIDTH);
	draw_robot(orange, R2_s_x, R2_s_y, ROBOT_HEIGHT, ROBOT_WIDTH);

	/* Re-draw robot's actual positions. */
	draw_robot(blue, robot1.r_px, robot1.r_py, robot1.r_height, robot1.r_width);
	draw_robot(orange, robot2.r_px, robot2.r_py, robot2.r_height, robot2.r_width);

	//Update robot positions in graphlist[movingnodes_ids[1 & 2]
	graphlist[movingnodes_id[0]].v_px = robot1.r_px;
	graphlist[movingnodes_id[0]].v_py = robot1.r_py;
	graphlist[movingnodes_id[1]].v_px = robot2.r_px;
	graphlist[movingnodes_id[1]].v_py = robot2.r_py;
	
	#ifdef DRAWNODES
	for(int sd = 0; sd<=nodeid-3; sd++)	//Dont draw nodes on robots and request... just for better view.
	{	
		draw_nodes(graphlist[sd].v_px, graphlist[sd].v_py);
	}
	#endif
}

/* A draw a simple line. */
void draw_line(GC _gc, int _x1, int _y1, int _x2, int _y2)
{
	XDrawLine(display_ptr, win, _gc, _x1, _y1, _x2, _y2);
}

/* Draw a segment/edge. */
void draw_segment(GC _gc, int __id, int _isEdge)
{
	if(_isEdge)
	{
		struct Edge* temp1 =GetEdgeById(__id);		
		if(temp1!=NULL) 
		draw_line(_gc, temp1->_x1, temp1->_y1, temp1->_x2, temp1->_y2);
	}
	else
	{
		struct Seg* temp =GetSegById(__id);
		if(temp!=NULL) 
		draw_line(_gc, temp->_x1, temp->_y1, temp->_x2, temp->_y2);
	}
}

/* Draw the position of the robot. */
void draw_robot(GC _color, int _posx, int _posy, int _height, int _width)
{
     XFillArc(display_ptr, win, _color, _posx-_width/2, _posy-_height/2, _height, _width,0,360*64);
	 //XDrawArc(display_ptr, win, red, _posx-_width/2, _posy-_height/2, _height, _width,0,360*64);
}

/* Draw the request points. If we have a re-draw don't 
   add the request point to the history as we already 
   know that we should just re-draw the request points. */
void draw_request(GC _color, int _posx, int _posy, int _isReDraw)
{   
    if(_isReDraw == 0) 
    {  AddRequestPoint(_posx, _posy); }
    XFillArc( display_ptr, win, _color, _posx-7.5, _posy-7.5, 15, 15, 0, 360*64);
}

/* Draw nodes to the graph. Sometimes not needed. */
void draw_nodes(int _posx, int _posy)
{ 
	XFillArc( display_ptr, win, white, _posx-3.5, _posy-3.5, 7,7, 0, 360*64);
}

void drawstring(GC _scolor, int sposx, int sposy, char *text)
{
	XDrawString(display_ptr, win, _scolor, sposx, sposy, text, strlen(text));
}

void drawint(GC _scolor, int sposx, int sposy, int inttodraw)
{
		char outtxt[50];
		sprintf(outtxt,"%d", inttodraw);
		char *tx = outtxt;
		drawstring(_scolor, sposx, sposy, tx);
}

void drawdouble(GC _scolor, int sposx, int sposy, double inttodraw)
{
		char outtxt[50];
		sprintf(outtxt,"%.2f", inttodraw);
		char *tx = outtxt;
		drawstring(_scolor, sposx, sposy, tx);
}

/* Clear a certain area of the window. */
void ClearArea(int _psx, int _psy, int _wclear, int _hclear, int _riseExposeEvent)
{
	XClearArea(display_ptr, win, _psx, _psy, _wclear, _hclear, _riseExposeEvent);
}

void about_info()
{
	XDrawRectangle(display_ptr, win, white, 705, 0, 190, 80);
	drawstring(blue, 708, 15, "ROBOT 1 Distance: ");
	drawstring(orange_shadow, 708, 30, "ROBOT 2 Distance: ");
	XDrawRectangle(display_ptr, win, white, 0, 0, 700, 485);
	drawstring(blue, 708, 45, "Last Travel Cost: ");
	drawstring(orange_shadow, 708, 60, "Last Travel Cost: ");
	drawstring(green, 708, 75, "Travel difference: ");

	XDrawRectangle(display_ptr, win, wallcolor, 705, 85, 190, 80);
	drawstring(white, 708, 100, "WEIGHTS OF PREDICTION: ");
	drawstring(white, 708, 115, "Travel Diff. Weight    : ");
	drawstring(white, 708, 130, "Distance Between Weight: ");
	drawstring(white, 708, 145, "High Border D. Weight  : ");
	drawstring(white, 708, 160, "Low Border D. Weight   : ");
	drawdouble(green,860,115, disdiffWeight);
	drawdouble(green,860,130, distWeight);
	drawint(green,860,145, (int)simdistHighBorder);
	drawint(green,860,160, (int)simdisLowBorder);

	
	XDrawRectangle(display_ptr,win,white,705,405,190,80);
	drawstring(white, 708, 420, "Graph Search - Dijkstra");
	drawstring(white, 708, 435, "2 Server Online Optimization");
	drawstring(orange, 708, 450, "Advanced Algorithms");
	drawstring(orange, 708, 465, "Lecturer: Peter Brass");
	drawstring(green, 708, 480, "Author: Ejup Hoxha");
}

/* Initialize colors which we will mostly use. */
void GetColors(){
	/* To be able to draw on this window we need to create graphics context. */
	gc = XCreateGC(display_ptr, win, valuemask, &gc_values);
	XSetForeground(display_ptr, gc, BlackPixel(display_ptr, screen_num));
	XSetLineAttributes(display_ptr, gc, 4, LineSolid, CapRound, JoinRound);

	/* Color/Lines with width 1. */
	green = XCreateGC(display_ptr, win, valuemask, &gc_green_v);
	XSetLineAttributes(display_ptr, green, 2, LineSolid,CapRound, JoinRound);
	if( XAllocNamedColor( display_ptr, color_map, "SpringGreen", &tmp_color1, &tmp_color2 ) == 0 )
    	{   
            printf("failed to get color Spring Green\n"); 
	    exit(-1);
	} 
  	else
   	    XSetForeground( display_ptr, green, tmp_color1.pixel );

	red = XCreateGC(display_ptr, win, valuemask, &gc_red_v);
	XSetLineAttributes(display_ptr, red, 1, LineSolid,CapRound, JoinRound);
	if( XAllocNamedColor( display_ptr, color_map, "red", &tmp_color1, &tmp_color2 ) == 0 )
    	{   
            printf("failed to get color red\n"); 
	    exit(-1);
	} 
  	else
   	    XSetForeground( display_ptr, red, tmp_color1.pixel );

	blue = XCreateGC(display_ptr, win, valuemask, &gc_blue_v);
	XSetLineAttributes(display_ptr, blue, 1, LineSolid,CapRound, JoinRound);
	if( XAllocNamedColor( display_ptr, color_map, "DeepSkyBlue", &tmp_color1, &tmp_color2 ) == 0 )
    	{   
            printf("failed to get color DeepSkyBlue\n"); 
	    exit(-1);
	}
  	else
	    XSetForeground( display_ptr, blue, tmp_color1.pixel );
	
	blue_ghost = XCreateGC(display_ptr, win, valuemask, &blue_sv);
	XSetLineAttributes(display_ptr, blue_ghost, 1, LineSolid,CapRound, JoinRound);
	if( XAllocNamedColor( display_ptr, color_map, "LightSkyBlue", &tmp_color1, &tmp_color2 ) == 0 )
    	{   
            printf("failed to get color blue-shadow\n"); 
	    exit(-1);
	}   	
	else
	    XSetForeground( display_ptr, blue_ghost, tmp_color1.pixel );

	black = XCreateGC(display_ptr, win, valuemask, &gc_black_v);
	XSetLineAttributes(display_ptr, black, 1, LineSolid,CapRound, JoinRound);
	if( XAllocNamedColor( display_ptr, color_map, "Blue", &tmp_color1, &tmp_color2 ) == 0 )
    	{   
            printf("failed to get color Black\n"); 
	    exit(-1);
	}
  	else
	    XSetForeground( display_ptr, black, tmp_color1.pixel );

	orange = XCreateGC(display_ptr, win, valuemask, &gc_orange_v);
	XSetLineAttributes(display_ptr, orange, 1, LineSolid,CapRound, JoinRound);
	if( XAllocNamedColor( display_ptr, color_map, "OrangeRed", &tmp_color1, &tmp_color2 ) == 0 )
    	{   
            printf("failed to get color orange\n"); 
	    exit(-1);
	} 
  	else
	    XSetForeground( display_ptr, orange, tmp_color1.pixel );

	orange_shadow = XCreateGC(display_ptr, win, valuemask, &orange_sv);
	XSetLineAttributes(display_ptr, orange_shadow, 1, LineSolid,CapRound, JoinRound);
	if( XAllocNamedColor( display_ptr, color_map, "Coral", &tmp_color1, &tmp_color2 ) == 0 )
    	{   
            printf("failed to get color orange-shadow\n"); 
	    exit(-1);
	} 
  	else
	    XSetForeground( display_ptr, orange_shadow, tmp_color1.pixel );

	white = XCreateGC(display_ptr, win, valuemask, &gc_white_v);
	XSetLineAttributes(display_ptr, white, 1, LineSolid,CapRound, JoinRound);
	if( XAllocNamedColor( display_ptr, color_map, "white", &tmp_color1, &tmp_color2 ) == 0 )
    	{   
            printf("failed to get color white\n"); 
	    exit(-1);
	} 
  	else
	    XSetForeground( display_ptr, white, tmp_color1.pixel );

	yellow = XCreateGC(display_ptr, win, valuemask, &gc_yellow_v);
	XSetLineAttributes(display_ptr, yellow, 1, LineSolid,CapRound, JoinRound);
	if( XAllocNamedColor( display_ptr, color_map, "Yellow", &tmp_color1, &tmp_color2 ) == 0 )
    	{   
            printf("failed to get color yellow\n"); 
	    exit(-1);
	} 
  	else
	    XSetForeground( display_ptr, yellow, tmp_color1.pixel );

	/* Color/Lines with line width = 3. */
	green_3 = XCreateGC(display_ptr, win, valuemask, &gc_green_v3);
	XSetLineAttributes(display_ptr, green_3, 3, LineSolid,CapRound, JoinRound);
	if( XAllocNamedColor( display_ptr, color_map, "SpringGreen", &tmp_color1, &tmp_color2 ) == 0 )
    	{   
            printf("failed to get color Spring Green\n"); 
	    exit(-1);
	} 
  	else
   	    XSetForeground( display_ptr, green_3, tmp_color1.pixel );

	blue_2 = XCreateGC(display_ptr, win, valuemask, &gc_blue_v2);
	XSetLineAttributes(display_ptr, blue_2, 2, LineSolid,CapRound, JoinRound);
	if( XAllocNamedColor( display_ptr, color_map, "DeepSkyBlue", &tmp_color1, &tmp_color2 ) == 0 )
    	{   
            printf("failed to get color DeepSkyBlue\n"); 
	    exit(-1);
	}
  	else
	    XSetForeground( display_ptr, blue_2, tmp_color1.pixel );

	orange_2 = XCreateGC(display_ptr, win, valuemask, &gc_orange_v2);
	XSetLineAttributes(display_ptr, orange_2, 2, LineSolid,CapRound, JoinRound);
	if( XAllocNamedColor( display_ptr, color_map, "OrangeRed", &tmp_color1, &tmp_color2 ) == 0 )
    	{   
            printf("failed to get color orange\n"); 
	    exit(-1);
	} 
  	else
	    XSetForeground( display_ptr, orange_2, tmp_color1.pixel );

	white_3 = XCreateGC(display_ptr, win, valuemask, &gc_white_v3);
	XSetLineAttributes(display_ptr, white_3, 3, LineSolid,CapRound, JoinRound);
	if( XAllocNamedColor( display_ptr, color_map, "white", &tmp_color1, &tmp_color2 ) == 0 )
    	{   
            printf("failed to get color blue\n"); 
	    exit(-1);
	} 
  	else
	    XSetForeground( display_ptr, white_3, tmp_color1.pixel );

	wallcolor = XCreateGC(display_ptr, win, valuemask, &wallcolor_v);
	XSetLineAttributes(display_ptr, wallcolor, 2, LineSolid,CapRound, JoinRound);
	if( XAllocNamedColor( display_ptr, color_map, "DeepPink", &tmp_color1, &tmp_color2 ) == 0 )
    	{   
            printf("failed to get color of wall\n"); 
	    exit(-1);
	} 
  	else
	    XSetForeground( display_ptr, wallcolor, tmp_color1.pixel );

}

#pragma endregion

#pragma region INPUT Decoding

void CheckStringType(char tmpstr[], int lencheck)
{
	char lastchar;
	for(int strcount = 0; strcount <lencheck; strcount++)
	{
		
		switch (tmpstr[strcount])
		{
			case '0':
				lastchar = '0';
				break;
			case '1':
				lastchar = '0';
				break;
			case '2':
				lastchar = '0';
				break;
			case '3':
				lastchar = '0';
				break;
			case '4':
				lastchar = '0';
				break;
			case '5':
				lastchar = '0';
				break;
			case '6':
				lastchar = '0';
				break;
			case '7':
				lastchar = '0';
				break;
			case '8':
				lastchar = '0';
				break;
			case '9':
				lastchar = '0';
				break;
			case '(':
				if(lastchar == '(')
				{
					printf("Bad format.\nDouble '%c' find and remove it\n", tmpstr[strcount]);
					exit(-1);
				}
				else if(lastchar == ')' ||lastchar == ',')
				{
					printf("Bad format.\nUnspecified error.\n");
					exit(-1);
				}
				lastchar = tmpstr[strcount];
				break;
			case ')':
				if(lastchar == ')')
				{
					printf("Bad format.\nDouble '%c' find and remove it\n", tmpstr[strcount]);
					exit(-1);
				}
				else if(lastchar == '(' || lastchar == ',' || lastchar == '-')
				{
					printf("Bad format.\nUnspecified error.\n");
					exit(-1);
				}
				lastchar = tmpstr[strcount];
				break;
			case '-':
				if(lastchar == '-')
				{
					printf("Bad format.\nDouble '%c' find and remove it\n", tmpstr[strcount]);
					exit(-1);
				}	
				else if(lastchar == '(' || lastchar == ',')
				{
					printf("Bad format.\nUnspecified error.\n");
					exit(-1);
				}			
				lastchar = tmpstr[strcount];
				break;
			case ',':
				if(lastchar == ',')
				{
					printf("Bad format.\nDouble '%c' find and remove it\n", tmpstr[strcount]);
					exit(-1);
				}
				else if(lastchar == '(' || lastchar == ')' || lastchar == '-')
				{
					printf("Bad format.\nUnspecified error.\n");
					exit(-1);
				}
				
				lastchar = tmpstr[strcount];
				break;
			case '\n':
				lastchar = tmpstr[strcount];	
				break;
			default:
				printf("Bad format.\nFind this character '%c' and remove it\n", tmpstr[strcount]);
				printf("Spaces and tabs aren't allowed.\n");
				exit(-1);
				break;
		}
	}
	
}
void text_to_segment(char * _buf, int seg_id)
{
	int _x_count = -1;	/* Start from 1st segment. */
	int _y_count = -1;	/* Start from 1st segment. */
	int _x[2] = { 0, 0};/* x coordinates corresponding to segment number.*/
	int _y[2] = {0 ,0};		/* y coordinates corresponding to segment number.*/
	char tmp[30];
	char tmp_y[30];
	
	CheckStringType(_buf,strlen(_buf));
	for(int k = 0; k<=strlen(_buf); k++)
	{
		if(_buf[k] == '(')
		{
		    if(_buf[k+1] != ',')
		    {
				k++;
				tmp[0] = _buf[k];
				if(_buf[k+1] != ',')
				{
					k++; tmp[1] = _buf[k]; 
					if(_buf[k+1] != ',')
					{
						k++; tmp[2] = _buf[k];
						if(_buf[k+1] != ',')
						{ k++; tmp[3] = _buf[k];}
					}
				}
   		    }
	        _x_count ++; _x[_x_count] = atoi(tmp); 
		}
		else if(_buf[k]==',')
		{     k++;
		      tmp_y[0] = _buf[k];
		      if(_buf[k+1] != ')')
		      {
				k++; tmp_y[1] = _buf[k]; 
				if(_buf[k+1] != ')')
				{
					k++; tmp_y[2] = _buf[k];
					if(_buf[k+1] != ')')
					{ 
							k++; tmp_y[3] = _buf[k];
					}	  
				}
		      }
   		     _y_count++; _y[_y_count] = atoi(tmp_y); 
		}
	}
	if(_x[0] >XBORDER || _x[1] >XBORDER || _y[0] > YBORDER || _y[1] >YBORDER)
	{
		printf("Segment coordinates outside the borders!");
		printf("\nBe aware that segments should fit inside: [%d x %d]",XBORDER, YBORDER);
		printf("\nCheck input file!\n");
		exit(-1);
	}
	AddSegment(seg_id, _x[0], _y[0], _x[1], _y[1]);
}

#ifdef SIMULATEDGRAPH
/* Simulate reading from file. 
   When you don't wanna use readings from file. */
void SimulateReadingFile()
{
	AddSegment(0, 40, 40, 140, 40);
	AddSegment(1, 90, 20, 90, 180);
	AddSegment(2, 70, 170, 200, 170);
	AddSegment(3, 190, 190, 190, 140);
	AddSegment(4, 220, 60, 220, 360);
	AddSegment(5, 250, 100, 250, 400);
	AddSegment(6, 240, 300, 290, 300);
	AddSegment(7, 280, 270, 280, 330);
	AddSegment(8, 270, 310, 350, 310);
	AddSegment(9, 400, 250, 400, 450);
	AddSegment(10, 430, 300, 430, 450);
	AddSegment(11, 490, 200, 490, 400);
	AddSegment(12, 390, 390, 495, 390);
	AddSegment(13, 390, 100, 420, 100);
	AddSegment(14, 440, 100, 470, 100);
	AddSegment(15, 410, 80, 460, 80);
	AddSegment(16, 430, 50, 430, 110);
	AddSegment(17, 360, 40, 440, 40);
	AddSegment(18, 30, 200, 30, 300);
	AddSegment(19, 20, 280, 50, 280);
	AddSegment(20, 40, 260, 40, 360);
	AddSegment(21, 30, 330, 60, 330);
	AddSegment(22, 50, 310, 50, 400);
	AddSegment(23, 20, 390, 60, 390);
}
#endif

#pragma endregion
