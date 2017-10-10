//
//  mmmaze.cpp
//  mmmaze
//
//  Created by Loy Kyle Wong on 11/09/2013.
//  Copyright (c) 2013 Loy Kyle Wong. All rights reserved.
//

#include "mmaze/mmaze.h"
#include "../TskIr/IrCorr.h"

using namespace std;

//#define SAFE

//#define abs(x) ((x) < 0? -(x) : (x))
namespace Micromouse
{

    void operator++(Direction::Type& l)
    {
        signed char i = (signed char)l + 1;
        if(i > (signed char)Direction::West)
            i -= 4;
        l = (Direction::Type)i;
    }
    
    Direction::Type operator+(const Direction::Type & l, const Turning::Type & r)
    {
        signed char rtn = (signed char)l + (signed char)r;
        if(rtn > 1) return (Direction::Type)(rtn - 4);
        else if(rtn < -2) return (Direction::Type)(rtn + 4);
        else return (Direction::Type)rtn;
    }
    
    const Direction::Type &operator+=(Direction::Type & l, const Turning::Type & r)
    {
        signed char rtn = (signed char)l + (signed char)r;
        if(rtn > 1) l = (Direction::Type)(rtn - 4);
        else if(rtn < -2) l = (Direction::Type)(rtn + 4);
        else l = (Direction::Type)rtn;
        return l;
    }
    
    inline Turning::Type operator-(const Direction::Type & l, const Direction::Type & r)
    {
        signed char rtn = (signed char)l - (signed char)r;
        if(rtn > 1) return (Turning::Type)(rtn - 4);
        if(rtn < -2) return (Turning::Type)(rtn + 4);
        else return (Turning::Type)rtn;
    }
    
    // N N N
    // W N E
    // S S S
    inline Direction::Type operator-(const GridCoor & l, const GridCoor & r)
    {
        short x = l.X - r.X;
        short y = l.Y - r.Y;
        if(abs(x) > abs(y)) // east or west
        {
            if (x < 0)
                return Direction::West;
            else
                return Direction::East;
        }
        else // north or south
        {
            if(y < 0)
                return Direction::South;
            else
                return Direction::North;
        }
    }
    
    GridCoor operator+(const GridCoor & l, const Direction::Type & r)
    {
        switch (r) {
            case Direction::East:
                return GridCoor(l.X + 1, l.Y);
            case Direction::West:
                return GridCoor(l.X - 1, l.Y);
            case Direction::North:
                return GridCoor(l.X, l.Y + 1);
            case Direction::South:
                return GridCoor(l.X, l.Y - 1);
            default:
                break;
        }
    	return GridCoor(l.X, l.Y);
    }
    
    const GridCoor &operator+=(GridCoor & l, const Direction::Type & r)
    {
        switch (r) {
            case Direction::East:
                l.X++;
                break;
            case Direction::West:
                l.X--;
                break;
            case Direction::North:
                l.Y++;
                break;
            case Direction::South:
                l.Y--;
                break;
            default:
                break;
        }
        return l;
    }

    MMaze::MMaze(short colNum, short rowNum, const GridCoor &target)
    {
        this->colNum = colNum;
        this->rowNum = rowNum;
        this->target = GridCoor(target);
        this->grids = new GridInfo[(colNum + 1) * (rowNum + 1)];
        for (int y = 0; y <= rowNum; y++)
        {
            for (int x = 0; x <= colNum; x++)
            {
                this->grids[index(x, y)].South = WallType::Undefined;

                this->grids[index(x, y)].West = WallType::Undefined;
            }
        }
    }

    MMaze::~MMaze()
    {
        delete[] this->grids;
    }
    
    inline int MMaze::index(const GridCoor &coor)
    {
        return coor.X + coor.Y * (this->colNum + 1);
    }
    
    inline int MMaze::index(short x, short y)
    {
        return x + y * (this->colNum + 1);
    }

    short MMaze::ColNum()
    {
        return this->colNum;
    }
    
    short MMaze::RowNum()
    {
        return this->rowNum;
    }
    
    GridCoor MMaze::Target()
    {
        return this->target;
    }
    
    bool MMaze::SetWall(const GridCoor& coor, Direction::Type dir, WallType::Type wall)
    {
#ifdef SAFE
        if(coor.X >= colNum || coor.X < 0 || coor.Y >= rowNum || coor.Y < 0)
            throw "Error: Coordinates exceed limit.";
#endif
        if(wall == WallType::Undefined) return true;
        switch (dir)
        {
        case Direction::South:
            this->grids[index(coor)].South = wall;
            break;
        case Direction::West:
            this->grids[index(coor)].West = wall;
            break;
        case Direction::North:
            this->grids[index(coor + Direction::North)].South = wall;
            break;
        case Direction::East:
            this->grids[index(coor + Direction::East)].West = wall;
            break;
        default:
            break;
        }
        return true;
    }
    
    bool MMaze::SetWall(const GridCoor& coor, WallType::Type north, WallType::Type west, WallType::Type south, WallType::Type east)
    {
#ifdef SAFE
        if(coor.X >= colNum || coor.X < 0 || coor.Y >= rowNum || coor.Y < 0)
            throw "Error: Coordinates exceed limit.";
#endif
        if(north != WallType::Undefined)
        {
            this->grids[index(coor + Direction::North)].South = north;
        }
        if(east != WallType::Undefined)
        {
            this->grids[index(coor + Direction::East)].West = east;
        }
        if(south != WallType::Undefined)
        {
            this->grids[index(coor)].South = south;
        }
        if(west != WallType::Undefined)
        {
            this->grids[index(coor)].West = west;
        }
        return true;
    }

    WallType::Type MMaze::GetWall(const GridCoor& coor, Direction::Type dir)
    {
#ifdef SAFE
        if(coor.X >= colNum || coor.X < 0 || coor.Y >= rowNum || coor.Y < 0)
            throw "Error: Coordinates exceed limit.";
#else
        if(coor.X >= colNum || coor.Y >= rowNum)
            return WallType::Undefined;
#endif
        switch (dir) {
            case Direction::South:
                return this->grids[index(coor)].South;
            case Direction::West:
                return this->grids[index(coor)].West;
            case Direction::North:
                return this->grids[index(coor + Direction::North)].South;
            case Direction::East:
                return this->grids[index(coor + Direction::East)].West;
            default:
                return WallType::Undefined;
        }
    }
    bool MMaze::StoreMazeGrids(void)
    {
    	TskIr::programFlash(254 * 1024,(unsigned int*)&this->grids[0],sizeof(GridInfo) * (colNum + 1) * (rowNum + 1));
        return 1;
    }

    bool MMaze::ReadMazeGrids(void)
    {
        TskIr::ReadFlash(0x3F800,(unsigned char*)&this->grids[0],sizeof(GridInfo) * (colNum + 1) * (rowNum + 1));
        return 1;
    }
    void MMaze::Fluid(unsigned short *height, const GridCoor& start, const GridCoor& target, WallType::Type wallCanGo, bool fullFluid)
    {
        static Queue<GridCoor> q(this->colNum * this->rowNum / 2);
        q.Reset();
        
        short h;
        GridCoor grid(0, 0);
        GridCoor adjGrid(0, 0);
        for(int y = 0; y < this->rowNum; y++)
            for(int x = 0; x < this->colNum; x++) {
                height[x + y * this->colNum] = 0;
            }
        q.Enqueue(start);
        height[start.X + start.Y * this->colNum] = 1;
        while (q.Length() > 0)
        {
        	grid = q.Dequeue();
            h = height[grid.X + grid.Y * this->colNum];
            for(signed char i = -2; i <= 1; i++)
            {
                adjGrid = grid + (Direction::Type)i;
                if(adjGrid.X >= this->colNum || adjGrid.Y >= this->rowNum)
                    continue;
                if(this->GetWall(grid, (Direction::Type)i) >= wallCanGo && height[adjGrid.X + adjGrid.Y * this->colNum] == 0)
                {
                    if(fullFluid || adjGrid.X != target.X || adjGrid.Y != target.Y) {
                        q.Enqueue(adjGrid);
                        height[adjGrid.X + adjGrid.Y * this->colNum] = h + 1;
                    }
                    else {
                        return;
                    }
                }
            }
        }
    }

}
