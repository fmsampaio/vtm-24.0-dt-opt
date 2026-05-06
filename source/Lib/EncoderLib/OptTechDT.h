#include <set>

#include "CommonLib/Picture.h"
#include "CommonLib/TimeProfiler.h"

#define ENABLE_OPT_TECH_DT 1

#define DBG_REPORT_DEPTH_MAPS 0
#define DBG_REPORT_VARIANCE 0

#define DEPTH_MAP_RESOLUTION 4
#define DEPTH_MAP_MAX_FRAME_SIZE ((3840 * 2160) / (DEPTH_MAP_RESOLUTION * DEPTH_MAP_RESOLUTION))
#define DEPTH_MAP_NUM_OF_FRAMES 33

#define ENCODER_RA_CONFIG 0
#define ENCODER_LD_CONFIG 1

const int REFERENCE_FRAME_ORDER[2][DEPTH_MAP_NUM_OF_FRAMES] = {
    {-1, 2, 4, 2, 8, 6, 4, 6, 16, 10, 8, 10, 8,  14, 12, 14, 32, 18, 20, 18, 24, 22, 20, 22, 16, 26, 28, 26, 24, 30, 28, 30, 0},
    {-1, 0, 1, 2, 3, 4, 5, 6, 7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31}
};

class OptTechDT {
    private:
        static int width, height, numOfFrames, depthMapAllocSize;
        static int depthMaps[DEPTH_MAP_NUM_OF_FRAMES][DEPTH_MAP_MAX_FRAME_SIZE];
        

    public:
        static int quantPar;
        static int encoderConfig;
        static bool skipCheckRD;

        static void init(int w, int h, int nf, std::string encCfg, int qp);

        static double calculateDiffVariance(int xBlk, int yBlk, int wBlk, int hBlk, PelUnitBuf origBuff, PelUnitBuf recoBuff);
        static double calculateBlockVariance(int xBlk, int yBlk, int wBlk, int hBlk, PelUnitBuf origBuff);
        static void debugVarianceCalculation(int xBlk, int yBlk, int wBlk, int hBlk, PelUnitBuf origBuff, PelUnitBuf recoBuff);

        static void updateDepthMap(int framePoc, int xBlk, int yBlk, int wBlk, int hBlk, int depth);
        static void reportDepthMap(int framePoc);

        static int isPreviousSplit(int refFramePoc, int xCU, int yCU, int currDepth);

        static PelUnitBuf getRefPicBuf(int currFramePoc, Slice* slice);
        static void reportRefPicsDbg();

        static void performModelDT(int currQtDepth, int ft_qp, double ft_bllockVar, double ft_diffVar, int ft_previousSplit, int ft_height, int ft_config);
};