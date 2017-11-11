//
//  mouse.cpp
//  mmmaze
//
//  Created by Loy Kyle Wong on 11/10/2013.
//  Copyright (c) 2013 Loy Kyle Wong. All rights reserved.
//

#include <math.h>

#include "action/action.h"
#include "mouse.h"
#include "mmaze/mmaze.h"
#include "../TskIr/IrCorr.h"
#include "solve/solve.h"


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
        TskIr::programFlash(62 * 1024 * 16 + 320,(unsigned int*)&this->searchedTimes[0],sizeof(unsigned short) * colNum * rowNum);
        return 1;
    }

    bool Mouse::ReadMazeInfo(void)
    {
        TskIr::ReadFlash(62 * 1024 * 16 + 320,(unsigned char*)&this->searchedTimes[0],sizeof(unsigned short) * colNum * rowNum);
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
            TskIr::eraseFlashBlock(62);
        	this->maze->StoreMazeGrids();
        	StoreMazeInfo();
        }
        else if(this->currCoor.X == 0 && this->currCoor.Y == 0)
        {
            this->state.Proc = MouseProcState::Running;
            *srchFinish = true;
            TskIr::eraseFlashBlock(62);
        	this->maze->StoreMazeGrids();
        	StoreMazeInfo();
        }

        return bestTurn;
    }

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
//   	    Turn2RushAct(&q);
    }
}
