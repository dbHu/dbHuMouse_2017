//
//  mouse.cpp
//  mmmaze
//
//  Created by Loy Kyle Wong on 11/10/2013.
//  Copyright (c) 2013 Loy Kyle Wong. All rights reserved.
//

#include "includes.h"
//using namespace std;

namespace Micromouse {

    Mouse::Mouse(short colNum, short rowNum, const GridCoor &target) {
		this->turnPrioritiesForwardFirst[0] = Turning::Forward;
		this->turnPrioritiesForwardFirst[1] = Turning::Right;
		this->turnPrioritiesForwardFirst[2] = Turning::Left;
		this->turnPrioritiesForwardFirst[3] = Turning::Backward;

        this->turnPriorities = this->turnPrioritiesForwardFirst;
		this->maze = new MMaze(colNum,rowNum,target);
        this->colNum = colNum;
        this->rowNum = rowNum;
        this->targetCoor = target;
        this->currCoor.Set(0, 0);
        this->currHeading = Direction::North;
        this->state.Proc = MouseProcState::Idle;
        this->state.RunTimes = 0;
        this->searchedTimes = new unsigned short[colNum * rowNum];
        this->height = new unsigned short[colNum * rowNum];

        for(int y = 0; y < rowNum; y++)
            for(int x = 0; x < colNum; x++)
            {
                this->maze->SetWall(GridCoor(x, y), WallType::Unknown, WallType::Unknown, WallType::Unknown, WallType::Unknown);
                if(x == 0)
                    this->maze->SetWall(GridCoor(x, y), Direction::West, WallType::Blocked);
                if(y == 0)
                    this->maze->SetWall(GridCoor(x, y), Direction::South, WallType::Blocked);
                if(x == 16)
                    this->maze->SetWall(GridCoor(x, y), Direction::East, WallType::Blocked);
                if(y == 16)
                    this->maze->SetWall(GridCoor(x, y), Direction::North, WallType::Blocked);
                this->searchedTimes[x + y * colNum] = 0;
            }
    }
    
    Mouse::~Mouse() {
        delete this->maze;
        delete[] this->height;
        delete[] this->searchedTimes;
    }
    
    inline int Mouse::index(const GridCoor &coor){
        return coor.X + coor.Y * this->colNum;
    }
    
    inline int Mouse::index(short x, short y){
        return x + y * this->colNum;
    }
    
    GridCoor Mouse::CurrCoor()
    {
        return this->currCoor;
    }
    
    Direction::Type Mouse::CurrHeading()
    {
        return this->currHeading;
    }

    short Mouse::searchedTimesConv(short t)
    {
    	float f = (float)t;
    	return (short)(0.5f + powf(f, 0.7f));	// 3 -> 2
//    	return (short)(0.5f + powf(f, 0.5f));	// 2 -> 1
//    	return (short)(0.5f + powf(f, 0.9f));	// 4 -> 3
    }

    bool Mouse::StoreMazeInfo(void)
    {
        TskIr::programFlash(254 * 1024 + 320,(unsigned int*)&this->searchedTimes[0],sizeof(unsigned short) * colNum * rowNum);
        return 1;
    }

    bool Mouse::ReadMazeInfo(void)
    {
        TskIr::ReadFlash(0x3F940,(unsigned char*)&this->searchedTimes[0],sizeof(unsigned short) * colNum * rowNum);
        return 1;
    }

    //TODO
    Turning::Type Mouse::Step(WallType::Type fwd, WallType::Type left, WallType::Type right, bool *srchFinish)
    {
        if(*srchFinish)
        {
        	this->maze->ReadMazeGrids();
        	ReadMazeInfo();
        	if(this->state.Proc != MouseProcState::Running)
        		*srchFinish = false;
        	this->state.Proc = MouseProcState::Running;
        }

        if(this->currCoor.X == 0 && this->currCoor.Y == 0 && this->state.Proc == MouseProcState::Idle)
            this->state.Proc = MouseProcState::FirstSearching;

        // TODO ...
        if(this->maze->GetWall(currCoor, currHeading + Turning::Forward) == WallType::Unknown)
        {
        	this->maze->SetWall(currCoor, currHeading + Turning::Forward, fwd);
        }
        if(this->maze->GetWall(currCoor, currHeading + Turning::Left) == WallType::Unknown)
        {
        	this->maze->SetWall(currCoor, currHeading + Turning::Left, left);
        }
        if(this->maze->GetWall(currCoor, currHeading + Turning::Right) == WallType::Unknown)
        {
        	this->maze->SetWall(currCoor, currHeading + Turning::Right, right);
        }
        //this->maze->IncrSearchedTime(currCoor);
        this->searchedTimes[index(currCoor)]++;
        switch (this->state.Proc)
        {
            case MouseProcState::FirstSearching:
                this->maze->Fluid(this->height, this->targetCoor, this->currCoor, WallType::Unknown, false);
                break;
            case MouseProcState::BackSearching:
                this->maze->Fluid(this->height, GridCoor(0, 0), this->currCoor, WallType::Unknown, false);
                break;
            case MouseProcState::Running:
                this->maze->Fluid(this->height, this->targetCoor, this->currCoor, WallType::Open, false);
                break;
            case MouseProcState::Idle:
//                throw "Step when Idle?";
            default:
                break;
        }
        
        int priority, mostPriority = 1000;
        Turning::Type bestTurn = Turning::Forward;
        Direction::Type dir;
        GridCoor adjGrid;
        bool flag = 1;
        for (int i = 0; i < 4; i++)
        {
            dir = currHeading + Mouse::turnPriorities[i];
            adjGrid = currCoor + dir;
            if (this->maze->GetWall(currCoor, dir) >= WallType::Open &&
                (priority = this->height[adjGrid.X + adjGrid.Y * this->colNum]) != 0)
            {
                if(this->state.Proc != MouseProcState::Running)
                {
                    //priority += this->maze->GetSearchedTimes(adjGrid) << 16;
                    priority += searchedTimesConv(this->searchedTimes[index(adjGrid)]);
                }
                if(priority < mostPriority)
                {
                    mostPriority = priority;
                    bestTurn = Mouse::turnPriorities[i];
                }
                flag = 0;
            }
        }
        if(flag)
        {
        	bestTurn = Turning::Backward;
        }
        if(bestTurn == Turning::Backward)
        {
            //this->maze->IncrSearchedTime(currCoor);
            this->searchedTimes[index(currCoor)]++;
        }
        this->currHeading += bestTurn;
        this->currCoor += this->currHeading;
        
        if(this->targetCoor.X == this->currCoor.X && this->targetCoor.Y == this->currCoor.Y)
        {
        	this->state.Proc = MouseProcState::BackSearching;
            TskIr::eraseFlashBlock(254);
        	this->maze->StoreMazeGrids();
        	StoreMazeInfo();
        }
        else if(this->currCoor.X == 0 && this->currCoor.Y == 0)
        {
            this->state.Proc = MouseProcState::Running;
            *srchFinish = true;
            TskIr::eraseFlashBlock(254);
        	this->maze->StoreMazeGrids();
        	StoreMazeInfo();
        }

        return bestTurn;
    }
#if 1
    void Turn2RushAct(Queue<Turning::Type> *q)
    {
    	int status = 0;
    	int p = 0;
    	Turning::Type turn;
    	while(q->Length())
    	{
    		turn = q->Dequeue();
    		switch(status)
    		{
    		case 0:	//Entry point
    			if(turn == Turning::Forward)
    				status = 1;
    			break;
    		case 1:
    			if(turn == Turning::Forward)
    				solve::QAct->En(TskAction::Act::CRush);
    			else if(turn == Turning::Right)
    				status = 2;
    			else if(turn == Turning::Left)
    				status = 3;
    			break;
    		case 2:
    			if(turn == Turning::Left)
    			{
    				solve::QAct->En(TskAction::Act::R45i);
    				status = 4;
    			}
    			else if(turn == Turning::Right)
    			{
    				status = 7;
    			}
    			else if(turn == Turning::Forward)
    			{
    				solve::QAct->En(TskAction::Act::R90r);
    				status = 1;
    			}
    			break;
    		case 3:
    			if(turn == Turning::Right)
    			{
    				solve::QAct->En(TskAction::Act::L45i);
    				status = 5;
    			}
    			else if(turn == Turning::Left)
    			{
    				status = 6;
    			}
    			else if(turn == Turning::Forward)
    			{
    				solve::QAct->En(TskAction::Act::L90r);
    				status = 1;
    			}
    			break;
    		case 4:
    			if(turn == Turning::Right)
    			{
    				solve::QAct->En(TskAction::Act::TRush);
    				status = 5;
    			}
    			else if(turn == Turning::Forward)
    			{
    				solve::QAct->En(TskAction::Act::L45o);
    				status = 1;
    			}
    			else if(turn == Turning::Left)
    			{
    				status = 8;
    			}
    			break;
    		case 5:
    			if(turn == Turning::Left)
    			{
    				solve::QAct->En(TskAction::Act::TRush);
    				status = 4;
    			}
    			else if(turn == Turning::Forward)
    			{
    				solve::QAct->En(TskAction::Act::R45o);
    				status = 1;
    			}
    			else if(turn == Turning::Right)
    			{
    				status = 9;
    			}
    			break;
    		case 6:
    			if(turn == Turning::Forward)
    			{
    				solve::QAct->En(TskAction::Act::L180);
    				status = 1;
    			}
    			else if(turn == Turning::Right)
    			{
    				solve::QAct->En(TskAction::Act::L135i);
    				status = 5;
    			}
    			break;
    		case 7:
    			if(turn == Turning::Forward)
    			{
    				solve::QAct->En(TskAction::Act::L180);
    				status = 1;
    			}
    			else if(turn == Turning::Left)
    			{
    				solve::QAct->En(TskAction::Act::R135i);
    				status = 4;
    			}
    			break;
    		case 8:
    			if(turn == Turning::Right)
    			{
    				solve::QAct->En(TskAction::Act::L90t);
    				status = 9;
    			}
    			else if(turn == Turning::Forward)
    			{
    				solve::QAct->En(TskAction::Act::L135o);
    				status = 1;
    			}
    			break;
    		case 9:
    			if(turn == Turning::Left)
    			{
    				solve::QAct->En(TskAction::Act::R90t);
    				status = 8;
    			}
    			else if(turn == Turning::Forward)
    			{
    				solve::QAct->En(TskAction::Act::L135o);
    				status = 1;
    			}
    			break;
    		}
    		p++;
    	}
    }
#endif
    void Mouse::GetQAct(void)
    {
    	Queue<Turning::Type> q(256);
    	this->maze->ReadMazeGrids();
    	ReadMazeInfo();
    	while((this->targetCoor.X != this->currCoor.X ) || (this->targetCoor.Y != this->currCoor.Y))
    	{
    		this->maze->Fluid(this->height, this->targetCoor, this->currCoor, WallType::Open, false);
            int priority, mostPriority = 1000;
            Turning::Type bestTurn = Turning::Forward;
            Direction::Type dir;
            GridCoor adjGrid;
            for (int i = 0; i < 4; i++)
            {
                dir = currHeading + Mouse::turnPriorities[i];
                adjGrid = currCoor + dir;
                if (this->maze->GetWall(currCoor, dir) >= WallType::Open &&
                    (priority = this->height[adjGrid.X + adjGrid.Y * this->colNum]) != 0)
                {

                    if(priority < mostPriority)
                    {
                        mostPriority = priority;
                        bestTurn = Mouse::turnPriorities[i];
                    }
                }
            }
            this->currHeading += bestTurn;
            this->currCoor += this->currHeading;
            q.Enqueue(bestTurn);
    	}
    	solve::QAct->En(TskAction::Act::RushIn);
    	Turn2RushAct(&q);
    	solve::QAct->En(TskAction::Act::RushOut);
    }
}
