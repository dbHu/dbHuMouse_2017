//
//  mouse.h
//  mmmaze
//
//  Created by Loy Kyle Wong on 11/10/2013.
//  Copyright (c) 2013 Loy Kyle Wong. All rights reserved.
//

#ifndef __mmmaze__mouse__
#define __mmmaze__mouse__

//#include <iostream>
#include "../mmaze/mmaze.h"

namespace Micromouse {
    
//    struct MouseStatus
//    {
//        GridCoor Position;
//        Direction Heading;
//        MouseStatus(const GridCoor &pos, Direction heading)
//        {
//            Position = pos;
//            Heading = heading;
//        }
//    };
    class MouseProcState
    {
    public : enum Type
        {
            Idle = 0,
            FirstSearching = 1,
            BackSearching = 2,
            Running = 3
        };
    };
    
    class MousePrintOption
    {
    public: enum Type
        {
            ShowNone = 0,
            ShowSearchedTimes = 1,
            ShowFluidHeights = 2
        };
    };
    
    struct MouseState
    {
        MouseProcState::Type Proc;
        short RunTimes;
        MouseState() {
            Proc = MouseProcState::Idle;
            RunTimes = 0;
        }
    };
    
    class Mouse {
    private:
        MMaze *maze;
        unsigned short *searchedTimes;
        unsigned short *searchedTimesMinor;
        unsigned short *height;
        short colNum, rowNum;
        GridCoor targetCoor;
        GridCoor currCoor;
        Direction::Type currHeading;
        MouseState state;
        Turning::Type turnPrioritiesForwardFirst[4];// = {Turning::Forward, Turning::Right, Turning::Left, Turning::Backward};
        Turning::Type turnPrioritiesLeftFirst[4];// = {Turning::Left, Turning::Forward, Turning::Right, Turning::Backward};
        Turning::Type turnPrioritiesRightFirst[4];// = {Turning::Right, Turning::Forward, Turning::Left, Turning::Backward};
        Turning::Type *turnPriorities;
        int index(const GridCoor &coor);
        int index(short x, short y);
        short searchedTimesConv(short t);
    public:
        Mouse(short colNum, short rowNum, const GridCoor &target);
        ~Mouse();
        GridCoor CurrCoor();
        Direction::Type CurrHeading();
        Turning::Type Step(WallType::Type fwd, WallType::Type left, WallType::Type right, bool *srchFinish);
        bool StoreMazeInfo(void);
        bool ReadMazeInfo(void);
        void GetQAct(void);
        //Turning::Type HwStep(WallType::Type fwd, WallType::Type left, WallType::Type right, bool *srchFinish);
        //void Print(stringstream &str, bool withTimes, bool withHeight, WallType wallCanGo);
        //void Print(stringstream &str, MousePrintOption::Type option, WallType::Type wallCanGo, GridCoor mouseHere = GridCoor(-1, -1));
        //const short *GetHeight();
        //const MMaze *GetMaze();
    };
}

#endif /* defined(__mmmaze__mouse__) */
