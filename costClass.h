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

#ifndef COSTCLASS_H_
#define COSTCLASS_H_

#include "anytimeRRT.h"
class MyCostClass: public CostClass
{public:
	MyCostClass(){}

	MyCostClass(ob::SpaceInformationPtr &si,int agent_num): CostClass::CostClass(si)
	{
		this->agent_num=agent_num;
		loadData();
	}

	virtual double cost(ob::State* state1, ob::State * state2 )
	{
		int agent_in_cost_area=0;//the number of agent that are in high-cost area
		const ompl::base::RealVectorStateSpace::StateType *state_1= state1->as<ompl::base::RealVectorStateSpace::StateType>();
	//	const ompl::base::RealVectorStateSpace::StateType *state_2= state2->as<ompl::base::RealVectorStateSpace::StateType>();

		for(int i=0;i<cost_area_vec.size();i++)
		{
			for(int j=0;j<agent_num;j++)
			{
				if(state_1->values[2*j]>cost_area_vec.at(i).x0
						&& state_1->values[2*j]<cost_area_vec.at(i).x1
						&& state_1->values[2*j+1]>cost_area_vec.at(i).y0
						&& state_1->values[2*j+1]<cost_area_vec.at(i).y1)
				{
					agent_in_cost_area++;
				}
			}

		}

		//the cost increases when the number of the agents that are in the high-cost area increases
		return si_->distance(state1,state2)*(1.0+double(agent_in_cost_area)/double(agent_num));
	}
	virtual ~MyCostClass(){}

	void loadData()
	    {
	    	FILE *p;
	    	char temp[30];
	    	char c='a';
	    	CostArea cost_area;

	    	if((p=fopen("cost_area.mp","r"))==NULL)
	    	{
	    		printf("cannot open cost_area files. using the uniform-cost environment now\n");
	    		return;
	    		//exit(0);

	    	}

	    	while(c!=EOF)
	    	{
	      		fscanf(p,"%lf %lf %lf %lf",&cost_area.x0,&cost_area.y0,&cost_area.x1,&cost_area.y1);

	    		c=fgetc(p);

	    		cost_area_vec.push_back(cost_area);
	    	}

	    	fclose(p);
	    }


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
	    int agent_num;
};



#endif /* COSTCLASS_H_ */
