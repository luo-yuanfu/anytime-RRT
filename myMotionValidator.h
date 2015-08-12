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

#ifndef MYMOTIONVALIDATOR_H_
#define MYMOTIONVALIDATOR_H_

#include "anytimeRRT.h"
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/MotionValidator.h>
#include <cmath>
#include <iostream>
#include <fstream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class myMotionValidator : public ob::MotionValidator
{
public:
	myMotionValidator(const ob::SpaceInformationPtr &si,int agent_num) : ob::MotionValidator(si)
	{
		this->agent_num=agent_num;
		loadData();
	}

    // implement checkMotion()
	bool checkMotion(const ob:: State *s1, const ob:: State *s2) const
	{
		const ompl::base::RealVectorStateSpace::StateType *state_1= s1->as<ompl::base::RealVectorStateSpace::StateType>();
		const ompl::base::RealVectorStateSpace::StateType *state_2= s2->as<ompl::base::RealVectorStateSpace::StateType>();

		for(int i=0;i<agent_num;i++)
		{
			double &x1(state_1->values[2*i]), &y1(state_1->values[2*i+1]), &x2(state_2->values[2*i]), &y2(state_2->values[2*i+1]);
			if(edgeCollision(x1,y1,x2,y2)) return false;
		}


		return true;
	}

	bool checkMotion(const ompl::base::State* s1, const ompl::base::State* s2, std::pair<ompl::base::State*, double>&ss) const
	{
		const ompl::base::RealVectorStateSpace::StateType *state_1= s1->as<ompl::base::RealVectorStateSpace::StateType>();
		const ompl::base::RealVectorStateSpace::StateType *state_2= s2->as<ompl::base::RealVectorStateSpace::StateType>();

		for(int i=0;i<agent_num;i++)
		{
			double &x1(state_1->values[2*i]), &y1(state_1->values[2*i+1]), &x2(state_2->values[2*i]), &y2(state_2->values[2*i+1]);
			if(edgeCollision(x1,y1,x2,y2)) return false;
		}


		return true;
	}

    bool myxor(bool a, bool b) const
    {
    	return ((a && !b) || (!a && b));
    }

    bool isCCW(double x0,double y0, double x1, double y1, double x2,double y2) const
    {
    	vect u,v;
    	u.setValue(x1-x0,y1-y0);
    	v.setValue(x2-x1,y2-y1);
    	return (u.x*v.y - v.x*u.y)>0;
    }

    bool isCoincident(double x0,double y0, double x1, double y1, double x2,double y2, double x3, double y3 ) const
    {
    	return myxor(isCCW(x0,y0,x1,y1,x2,y2),
    	isCCW(x0,y0,x1,y1,x3,y3)) &&
    	myxor(isCCW(x2,y2,x3,y3,x0,y0),
    	isCCW(x2,y2,x3,y3,x1,y1));
    }

    bool edgeCollision(double x0,double y0, double x1, double y1) const
    {
    	for(int i=0;i<obsts.size();i++)
    	{
    		if( isCoincident(x0,y0,x1,y1,obsts.at(i).x0,obsts.at(i).y0,obsts.at(i).x0,obsts.at(i).y1) ||
    			isCoincident(x0,y0,x1,y1,obsts.at(i).x0,obsts.at(i).y1,obsts.at(i).x1,obsts.at(i).y1) ||
    			isCoincident(x0,y0,x1,y1,obsts.at(i).x1,obsts.at(i).y0,obsts.at(i).x1,obsts.at(i).y1) ||
    			isCoincident(x0,y0,x1,y1,obsts.at(i).x0,obsts.at(i).y0,obsts.at(i).x1,obsts.at(i).y0)) return true;

    	}

    	return false;
    }

    void loadData()
    {
    	FILE *p;
    	char temp[30];
    	char c='a';
    	obstacles obst;

    	if((p=fopen("obstacles.mp","r"))==NULL)
    	{
    		printf("cannot open obstacle files. using the environment without obstacles now\n");
    		return;
    		//exit(0);

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

    		obsts.push_back(obst);
    	}

    	fclose(p);
    }

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

    std::vector<obstacles> obsts;
    int agent_num;
};


#endif /* MYMOTIONVALIDATOR_H_ */
