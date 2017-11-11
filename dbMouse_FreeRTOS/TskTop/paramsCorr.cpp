#include "paramsCorr.h"

VarAndName varname[varnum] = {
	{&TskMotor::pidparam.avPidP,	"avpidp"},
	{&TskMotor::pidparam.avPidI,	"avpidi"},
	{&TskMotor::pidparam.posCoff,	"lqrpos"},
	{&TskMotor::pidparam.velCoff,	"lqrvel"},

    {&CP.EncoderUnitCompensation,	"encode"},
    {&CP.GyroUnitCompensation,	 	"gryo"},
    {&CP.AcclUnitCompensation,		"accl"},

    {&CP.LFWDEND_DIST_W2NW,			"lw2nw"},
    {&CP.RFWDEND_DIST_W2NW,			"rw2nw"},
    {&CP.CENTIPEDE_CORR_GAIN,		"pcenti"},

    {&CP.TURNL90_PRE_ADJ,			"l90pre"},
    {&CP.TURNR90_PRE_ADJ,			"r90pre"},
    {&CP.TURNL90_POST_ADJ,			"l90post"},
    {&CP.TURNR90_POST_ADJ, 			"r90post"}, 
    {&CP.TURNLWAIT_DIST_ADJ,		"l90wait"},
    {&CP.TURNRWAIT_DIST_ADJ,		"r90wait"},

    {&CP.RESTART_DIST_ADJ,			"restartadj"},

    {&CP.STOPEND_DIST_ADJ,			"stopadj"},

    {&CP.FWDDISADJ,					"backdist"},
    {&CP.LRBACKANGLE_ADJ,			"backangle"},
    {&CP.FLRYAWERROR,				"backyawerr"},

    {&SP.RushTurnSpeed,				"rspd"},
    {&SP.BackSpeed,					"bspd"},
    {&SP.RushDiagSpeed,				"diagspd"},
    {&SP.RushMu,                    "rmu"},
    {&SP.TL180Mu,                   "tlmu"},
    {&SP.TR180Mu,                   "trmu"},

    {&SP.ORushEndLDist,				"orushl"},
    {&SP.ORushEndRDist,				"orushr"},
    {&SP.DRushEndLDist,				"drushl"},
    {&SP.DRushEndRDist,				"drushr"},

   // 45 degree left & right turn in or out straight segment
    {&SP.TURNLI45_PRE_ADJ,			"l45ipre"},
    {&SP.TURNRI45_PRE_ADJ,			"r45ipre"},
    {&SP.TURNLI45_POST_ADJ,			"l45ipost"},
    {&SP.TURNRI45_POST_ADJ,			"r45ipost"},
    {&SP.TURNLO45_PRE_ADJ,			"l45opre"},
    {&SP.TURNRO45_PRE_ADJ,			"r45opre"},
    {&SP.TURNLO45_POST_ADJ,			"l45opost"},
    {&SP.TURNRO45_POST_ADJ,			"r45opost"},

    //the value about deciding the turn;
    {&SP.TURNLI45TT_ADJ ,			"l45iwait"},
    {&SP.TURNRI45TT_ADJ ,			"r45iwait"},
    {&SP.TURNLO45TT_ADJ ,			"l45owait"},
    {&SP.TURNRO45TT_ADJ ,			"r45owait"},

   // 90 degree left & right turn in straight segment(O90)
    {&SP.TURNLO90_PRE_ADJ ,			"l90opre"},		
    {&SP.TURNRO90_POST_ADJ ,		"r90opost"},
    {&SP.TURNLO90_PRE_ADJ ,			"l90opre"},
    {&SP.TURNRO90_POST_ADJ ,		"r90opost"},

   // 90 degree left & right turn in straight segment(D90)
    {&SP.TURNLD90_PRE_ADJ ,			"l90dpre"},
    {&SP.TURNRD90_PRE_ADJ ,			"r90dpre"},
    {&SP.TURNLD90_POST_ADJ ,		"l90dpost"},
    {&SP.TURNRD90_POST_ADJ ,		"r90dpost"},

    //the value about deciding the turn;
    {&SP.TURNLO90TT_ADJ ,			"l90owait"},
    {&SP.TURNRO90TT_ADJ ,			"r90owait"},
    {&SP.TURNLD90TT_ADJ ,			"l90dwait"},
    {&SP.TURNRD90TT_ADJ ,			"r90dwait"},

   // 135 degree left & right turn in straight segment
    {&SP.TURNLI135_PRE_ADJ ,		"l135ipre"},
    {&SP.TURNRI135_PRE_ADJ ,		"r135ipre"},
    {&SP.TURNLI135_POST_ADJ,		"l135ipost"},
    {&SP.TURNRI135_POST_ADJ,		"r135ipost"},

   //135 degree left & right turn out straight segment
    {&SP.TURNLO135_PRE_ADJ ,		"l135opre"},
    {&SP.TURNRO135_PRE_ADJ ,		"r135opre"},
    {&SP.TURNLO135_POST_ADJ,		"l135opost"},
    {&SP.TURNRO135_POST_ADJ,		"r135opost"},

    //the value about deciding the turn;
    {&SP.TURNLI135TT_ADJ ,			"l135iwait"},
    {&SP.TURNRI135TT_ADJ ,			"r135iwait"},
    {&SP.TURNLO135TT_ADJ ,			"l135owait"},
    {&SP.TURNRO135TT_ADJ ,			"r135owait"},

   // 180 degree left & right turn in straight segment
    {&SP.TURNL180_PRE_ADJ ,			"l180pre"},
    {&SP.TURNR180_PRE_ADJ ,			"r180pre"},
    {&SP.TURNL180_POST_ADJ,			"l180post"},
    {&SP.TURNR180_POST_ADJ,			"r180post"},

    {&SP.TURNL180TT_ADJ ,			"l180wait"},
    {&SP.TURNR180TT_ADJ ,			"r180wait"},

};

ActAndName actname[actnum] = {
    {TskAction::Act::Start,     "start"  },
    {TskAction::Act::Stop,      "stop"   },
    {TskAction::Act::Back,      "back"   },
    {TskAction::Act::Restart,   "restart"},
    {TskAction::Act::Fwd,       "fwd"    },
    {TskAction::Act::L90,       "l90"    },
    {TskAction::Act::R90,       "r90"    },
    {TskAction::Act::TBackR,    "tbackr" },

    {TskAction::Act::RushStart, "xstart" },
    {TskAction::Act::RushStop,  "xstop"  },
    {TskAction::Act::ORush,     "orush"  },
    {TskAction::Act::ORushAcc,  "orush+" },
    {TskAction::Act::ORushDea,  "orush-" },
    {TskAction::Act::DRush,     "drush"  },
    {TskAction::Act::DRushAcc,  "drush+" },
    {TskAction::Act::DRushDea,  "drush-" },
    {TskAction::Act::L45i,      "l45i"   },
    {TskAction::Act::L45o,      "l45o"   },
    {TskAction::Act::R45i,      "r45i"   },
    {TskAction::Act::R45o,      "r45o"   },
    {TskAction::Act::L90o,      "l90o"   },
    {TskAction::Act::R90o,      "r90o"   },
    {TskAction::Act::L90d,      "l90d"   },
    {TskAction::Act::R90d,      "r90d"   },
    {TskAction::Act::L135i,     "l135i"  },
    {TskAction::Act::L135o,     "l135o"  },
    {TskAction::Act::R135i,     "r135i"  },
    {TskAction::Act::R135o,     "r135o"  },
    {TskAction::Act::L180,      "l180"   },
    {TskAction::Act::R180,      "r180"   }
};
