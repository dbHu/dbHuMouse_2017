//
//  mmmaze.h
//  mmmaze
//
//  Created by Loy Kyle Wong on 11/09/2013.
//  Copyright (c) 2013 Loy Kyle Wong. All rights reserved.
//

#ifndef MMAZE_H_
#define MMAZE_H_

//#include <iostream>
//#include <sstream>
//#include <fstream>

using namespace std;

namespace Micromouse
{
    class WallType
    {
    public: enum Type
        {
            Undefined = 0,
            Blocked = 1,
            Unknown = 2,
            Open = 3
        };
    };

    class Direction
    {
    public : enum Type
        {
            North = 0,
            West = 1,
            South = -2,
            East = -1
        };
    };
    
    class Turning
    {
    public : enum Type
        {
            Forward = 0,
            Left = 1,
            Backward = -2,
            Right = -1
        };
    };

#pragma pack(push)
#pragma pack(2)
    struct GridCoor
    {
        // east x, north y
        unsigned short X : 5;
        unsigned short Y : 5;
        GridCoor() {
            X = 0; Y = 0;
        }
        GridCoor(unsigned short x, unsigned short y) {
            X = x; Y = y;
        }
        void Set(unsigned short x, unsigned short y) {
            X = x; Y = y;
        }
    };
#pragma pack(pop)
    
    //void (operator++)((Direction::Type)&);
    // direction turning
    Direction::Type operator+(const Direction::Type &, const Turning::Type &);
    const Direction::Type &operator+=(Direction::Type &, const Turning::Type &);
    Turning::Type operator-(const Direction::Type &, const Direction::Type &);
    // grid direction
    GridCoor operator+(const GridCoor &, const Direction::Type &);
    const GridCoor &operator+=(GridCoor &, const Direction::Type &);
    Direction::Type operator-(const GridCoor &, const GridCoor &);
    
#pragma pack(push)
#pragma pack(1)
    struct GridInfo
    {
        WallType::Type South : 2;
        WallType::Type West : 2;
        GridInfo()
        {
            South = WallType::Undefined;
            West = WallType::Undefined;
        }
    };
#pragma pack(pop)
    
    template <typename T>
    class Queue
    {
    private:
        T *array;
        int head;
        int tail;
        int capacity;
    public:
   Queue(int capacity)
    {
        this->array = new T[capacity + 1];
        this->head = 0;
        this->tail = 0;
        this->capacity = capacity;
    };
    ~Queue()
    {
        delete[] this->array;
    }
   void Enqueue(const T & data)
    {
#ifdef SAFE
        if (Length() < capacity)
        {
#endif
            this->array[tail++] = data;
            if (tail > capacity) tail = 0;
#ifdef SAFE
        }
        else
            throw "Error: Enqueue when queue full.";
#endif
    }

    T Dequeue()
    {
#ifdef SAFE
        if (Length() > 0)
        {
#endif
            T& rtn = this->array[head++];
            if(head > capacity) head = 0;
            return rtn;
#ifdef SAFE
        }
        else
            throw "Error: Dequeue when queue empty.";
#endif
    }

    T Peek()
    {
#ifdef SAFE
        if (Length() > 0)
        {
#endif
            return this->array[head++];
#ifdef SAFE
        }
        else
            throw "Error: Peek when queue empty";
#endif
    }

    void Reset()
    {
        this->head = 0;
        this->tail = 0;
    }

    int Length()
    {
        int rtn = this->tail - this->head;
        if(rtn < 0)
            return rtn + capacity + 1;
        else
            return rtn;
    }

    int Capacity()
    {
        return capacity;
    }
    };

    class MMaze
    {
    private:
        GridInfo *grids;
        GridInfo *hwGrids;
        short colNum;
        short rowNum;
        GridCoor target;
        int index(const GridCoor &coor);
        int index(short x, short y);
    public:
        MMaze(short colNum, short rowNum, const GridCoor &target);
//        MMaze(ifstream &file);
        ~MMaze();
//        void Read(ifstream file);
        bool SetWall(const GridCoor& coor, Direction::Type dir, WallType::Type wall);
        bool SetWall(const GridCoor& coor, WallType::Type north, WallType::Type west, WallType::Type south, WallType::Type east);
        WallType::Type GetWall(const GridCoor& coor, Direction::Type dir);
        void Fluid(unsigned short * height, const GridCoor &start, const GridCoor &target, WallType::Type wallCanGo, bool fullFluid);
       //unsigned short * HwFluid(const GridCoor &start, const GridCoor &target, WallType::Type wallCanGo);
       // void HwHeightClear();
        bool StoreMazeGrids(void);
        bool ReadMazeGrids(void);
        short ColNum();
        short RowNum();
        GridCoor Target();
//        void Print(stringstream &str, bool withTimes, WallType wallCanGo, GridCoor mouseHere = GridCoor(-1, -1));
//        void Print(stringstream &str, WallType::Type wallCanGo, unsigned short *info = NULL, GridCoor mouserHere = GridCoor(-1, -1));
    };
}
#endif /* defined(__mmmaze__mmmaze__) */
