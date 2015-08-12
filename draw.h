/*
 * anytimeRRT.cpp
 *
 *  Created on: Nov 8, 2014
 *      Author: Luo Yuanfu
 *  This Project is based on the paper:
	Ferguson, Dave, and Anthony Stentz. "Anytime rrts."
	In Intelligent Robots and Systems, 2006 IEEE/RSJ
	International Conference on, pp. 5369-5375. IEEE, 2006.
 */

#ifndef DRAW_H_
#define DRAW_H_

#include <graphics.h>
#include <X11/Xlib.h>


class vect//vector
{public:
    double x;
    double y;
    void setValue(double x,double y)
    {
    	this->x=x;
    	this->y=y;
    }
};

class obstacles
{public:
    double x0;
    double y0;
    double x1;
    double y1;

    void setValue(double x0,double y0,double x1,double y1)
    {
    	this->x0=x0;
    	this->y0=y0;
    	this->x1=x1;
    	this->y1=y1;
    }
};

class robot_pos
{public:
	double x1;
	double y1;
	double x2;
	double y2;
	double x3;
	double y3;

	void setValue(double x1,double y1,double x2,double y2,double x3, double y3)
	{
	    this->x1=x1;
	    this->y1=y1;
	    this->x2=x2;
	    this->y2=y2;
	    this->x3=x3;
	   	this->y3=y3;
	}
};

class CostArea
{public:
   	double x0;
   	double y0;
   	double x1;
   	double y1;
  	void setValue(double x0,double y0,double x1,double y1)
   	{
   		this->x0=x0;
   		this->y0=y0;
   		this->x1=x1;
   		this->y1=y1;
   	}
};

std::vector<CostArea> cost_area_vec;
std::vector<obstacles> obsts;
std::vector<robot_pos> path;

void drawPath(std::vector<robot_pos>path,double env_width,double env_height,int agent_num)
{
	int graph_width=getmaxx();
	int graph_height=getmaxy();

	setcolor(YELLOW);
	circle(int(path.at(0).x1/env_width*graph_width),int(path.at(0).y1/env_height*graph_height),3);

	for(int i=1;i<path.size();i++)
	{

		line(int(path.at(i).x1/env_width*graph_width),int(path.at(i).y1/env_height*graph_height),int(path.at(i-1).x1/env_width*graph_width),int(path.at(i-1).y1/env_height*graph_height));
		circle(int(path.at(i).x1/env_width*graph_width),int(path.at(i).y1/env_height*graph_height),3);
	}

	if(agent_num!=3) return;
	setcolor(GREEN);
	circle(int(path.at(0).x2/env_width*graph_width),int(path.at(0).y2/env_height*graph_height),3);

	for(int i=1;i<path.size();i++)
	{

		line(int(path.at(i).x2/env_width*graph_width),int(path.at(i).y2/env_height*graph_height),int(path.at(i-1).x2/env_width*graph_width),int(path.at(i-1).y2/env_height*graph_height));
		circle(int(path.at(i).x2/env_width*graph_width),int(path.at(i).y2/env_height*graph_height),3);
	}


	setcolor(RED);
	circle(int(path.at(0).x3/env_width*graph_width),int(path.at(0).y3/env_height*graph_height),3);

	for(int i=1;i<path.size();i++)
	{

		line(int(path.at(i).x3/env_width*graph_width),int(path.at(i).y3/env_height*graph_height),int(path.at(i-1).x3/env_width*graph_width),int(path.at(i-1).y3/env_height*graph_height));
		circle(int(path.at(i).x3/env_width*graph_width),int(path.at(i).y3/env_height*graph_height),3);
	}



}




void drawObsts(double env_width,double env_height)
{
	int graph_width=getmaxx();
	int graph_height=getmaxy();

	setcolor(WHITE);
//	setfillstyle(BROWN,5,NULL);
	for(int i=0;i<obsts.size();i++)
	{
		//fillpoly(2,int(obsts.at(i).x0*500),int(obsts.at(i).y0*500),int(obsts.at(i).x1*500),int(obsts.at(i).y1*500));
		rectangle(int(obsts.at(i).x0/env_width*graph_width),int(obsts.at(i).y0/env_height*graph_height),int(obsts.at(i).x1/env_width*graph_width),int(obsts.at(i).y1/env_height*graph_height));
		//bar(int(obsts.at(i).x0*graph_width),int(obsts.at(i).y0*graph_height),int(obsts.at(i).x1*graph_width),int(obsts.at(i).y1*graph_height));
	}
}

void drawCostArea(double env_width,double env_height)
{
	int graph_width=getmaxx();
	int graph_height=getmaxy();

	setcolor(BROWN);
//	setfillstyle(BROWN,5,NULL);
	for(int i=0;i<cost_area_vec.size();i++)
	{
		//fillpoly(2,int(obsts.at(i).x0*500),int(obsts.at(i).y0*500),int(obsts.at(i).x1*500),int(obsts.at(i).y1*500));
		rectangle(int(cost_area_vec.at(i).x0/env_width*graph_width),int(cost_area_vec.at(i).y0/env_height*graph_height),int(cost_area_vec.at(i).x1/env_width*graph_width),int(cost_area_vec.at(i).y1/env_height*graph_height));
		//bar(int(obsts.at(i).x0*graph_width),int(obsts.at(i).y0*graph_height),int(obsts.at(i).x1*graph_width),int(obsts.at(i).y1*graph_height));
	}
}

void loadData(int agent_num)
  {
  	FILE *p;
  	char temp[30];
  	char c='a';
  	obstacles obst;
  	robot_pos robot;
  	CostArea cost_area;

  	if((p=fopen("obstacles.mp","r"))==NULL)
  	{
  		//printf("cannot open obstacle files. using the environment without obstacles now\n");
  		//return;
  		goto out2;
  	}

  	while(c!=EOF)
  	{
  		fgets(temp,30,p);
  		fscanf(p,"%lf %lf",&obst.x0,&obst.y0);
  		fgetc(p);
  		fscanf(p,"%lf %lf",&obst.x0,&obst.y0);
  		fgetc(p);
  		fscanf(p,"%lf %lf",&obst.x1,&obst.y1);
  		fgetc(p);
  		fscanf(p,"%lf %lf",&obst.x1,&obst.y1);
  		fgetc(p);
  		fgets(temp,30,p);
  		c=fgetc(p);
  		/*
  		if(obst.x0>obst.x1)
  		{
  			double temp_swap=obst.x1;
  			obst.x1=obst.x0;
  			obst.x0=temp_swap;
  		}
  		if(obst.y0>obst.y1)
  		{
  		  	double temp_swap=obst.y1;
  		  	obst.y1=obst.y0;
  		  	obst.y0=temp_swap;
  		}
  		*/
  		obsts.push_back(obst);
  	}

out2:  	fclose(p);

  	if((p=fopen("path.dat","r"))==NULL)
  	{
  	  	printf("no solution file\n");
  	  	exit(0);

  	}

  	char c1='b';
    while(c1!=EOF)
    {

  	    if(agent_num==3)fscanf(p,"%lf %lf %lf %lf %lf %lf",&robot.x1,&robot.y1,&robot.x2,&robot.y2,&robot.x3,&robot.y3);
  	    else fscanf(p,"%lf %lf",&robot.x1,&robot.y1);
  	  //	fgetc(p);

  	  	c1=fgetc(p);

  	  	path.push_back(robot);
    }

   fclose(p);



   	if((p=fopen("cost_area.mp","r"))==NULL)
   	{
   		printf("cannot open cost_area files. using the uniform-cost environment now\n");
   		return;
   		//exit(0);
   	}
   	char c2='a';
   	while(c2!=EOF)
   	{
   		fscanf(p,"%lf %lf %lf %lf",&cost_area.x0,&cost_area.y0,&cost_area.x1,&cost_area.y1);

   	    c2=fgetc(p);

   	    cost_area_vec.push_back(cost_area);
   	}

   	fclose(p);


}

void drawMap(double env_width,double env_height,int agent_num)
{
	XInitThreads();
	int gd=DETECT,gm;
	initgraph(&gd,&gm,NULL);
//	initgraph(,500);

	loadData(agent_num);

	drawObsts(env_width,env_height);

	drawPath(path,env_width,env_height,agent_num);

	drawCostArea(env_width,env_height);

	getchar();

	closegraph();
}


#endif /* DRAW_H_ */
