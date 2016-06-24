/**
 * @file contour
 * @author Markus Bader
 * @date Mo Jul 9 2007
 * @version 0.1
 * @brief
 *
 * @see
 **/
#include "tuw_utils/contour.h"
#include <iostream>
#include <stack>
#include <cstdio>

namespace tuw {

const int Contour::ANGLE_8U = 8;
const int Contour::ANGLE_32F = 32;
const int Contour::ANGLE_64F = 64;


const int Contour::MODE_SIMPLE = 1;
const int Contour::MODE_CONTOUR = 2;
const int Contour::MODE_GRAIDENT = 3;
const int Contour::MODE_COMPLEX = 4;

const float Contour::fPi  = 3.1415926535897932384626433832795;
const double Contour::dPi = 3.1415926535897932384626433832795;

const int Contour::pppDirectionWeightsField[4][3][3] = {
    {   {2,1,2}, // 0 0 0
        {4,0,4}, // 1 1 1
        {2,1,2}  // 0 0 0
    },
    {   {1,2,4}, // 0 0 1
        {2,0,2}, // 0 1 0
        {4,2,1}  // 1 0 0
    },
    {   {2,4,2}, // 0 1 0
        {1,0,1}, // 0 1 0
        {2,4,4}  // 0 1 0
    },
    {   {4,2,1}, // 1 0 0
        {2,0,2}, // 0 1 0
        {1,2,4}  // 0 0 1
    }
};

const int Contour::ppGradientWeightsField[4][9] = {
    {3, 5, 6, 8, 2, 0, 7, 1, 4},
    {2, 6, 1, 3, 7, 5, 0, 8, 4},
    {1, 7, 0, 6, 8, 2, 3, 5, 4},
    {0, 8, 1, 7, 5, 3, 2, 6, 4}
};

const int Contour::pppCommingFromEdgeWeightsField[9][3][3] = {
    {   {0,1,2}, // 1 0 0
        {1,0,3}, // 0 0 0
        {2,3,4}  // 0 0 0
    },
    {   {1,0,1}, // 0 1 0
        {2,0,2}, // 0 0 0
        {3,4,3}  // 0 0 0
    },
    {   {2,1,0}, // 0 0 1
        {3,0,1}, // 0 0 0
        {4,3,2}  // 0 0 0
    },
    {   {1,2,3}, // 0 0 0
        {0,0,4}, // 1 0 0
        {1,2,3}  // 0 0 0
    },
    {   {0,0,0}, // 0 0 0
        {0,0,0}, // 0 1 0
        {0,0,0}  // 0 0 0
    },
    {   {3,2,1}, // 0 0 0
        {4,0,0}, // 0 0 1
        {3,2,1}  // 0 0 0
    },
    {   {2,3,4}, // 0 0 0
        {1,0,3}, // 0 0 0
        {0,1,2}  // 1 0 0
    },
    {   {3,4,3}, // 0 0 0
        {2,0,2}, // 0 0 0
        {1,0,1}  // 0 1 0
    },
    {   {4,3,2}, // 0 0 0
        {3,0,1}, // 0 0 0
        {2,1,0}  // 0 0 1
    }
};

const int Contour::ppContourWeightsField[9][9] = {
    {8, 7, 5, 6, 2, 3, 1, 0, 4},
    {7, 6, 8, 3, 5, 0, 2, 1, 4},
    {6, 7, 3, 8, 0, 5, 1, 2, 4},
    {5, 2, 8, 1, 7, 0, 6, 3, 4},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {3, 6, 0, 7, 1, 8, 2, 5, 4},
    {2, 1, 5, 0, 8, 3, 7, 6, 4},
    {1, 2, 0, 3, 5, 6, 8, 7, 4},
    {0, 1, 3, 2, 6, 5, 7, 8, 4}
};

Contour::Contour() :
    mpImgEdge ( NULL ),
    mpImgEdgeDirection ( NULL ),
    mdImgEdgeDirectionType ( 0 ),
    mdefEdgeLinkMode ( 0 ),
    mImgWidth ( 0 ),
    mImgHeight ( 0 ),
    mbAllowToModifyTheSources ( false ),
//            mpEdges ( NULL ),
    mNrOfEdges ( 0 ) {}

Contour::~Contour() {
    Contour::RelaseMemory();
}

void Contour::AllocateMemory() {
    RelaseMemory();
    if ( mbAllowToModifyTheSources == false ) {
        mpImgEdge = ( unsigned char* ) malloc ( sizeof ( unsigned char ) * mImgWidth * mImgHeight );
    }
    //mpEdges = ( CvPoint* ) malloc ( sizeof ( CvPoint ) * mImgWidth * mImgHeight );
    mEdges.resize ( mImgWidth * mImgHeight );
    mAngle8Bit.resize ( mImgWidth * mImgHeight );
}

void Contour::RelaseMemory() {
    if ( mbAllowToModifyTheSources ) {
        if ( mpImgEdge != NULL ) {
            free ( mpImgEdge );
            mpImgEdge = NULL;
        }
        /*
        if ( mpEdges != NULL ) {
        free ( mpEdges );
        mpEdges = NULL;
        }
        */
    }
}

int Contour::getContours( std::vector<std::vector<cv::Point> > &contours, unsigned int min_length) {
    contours.clear();
    std::vector<cv::Range> idx = getSegmentIndexes () ;
    for ( unsigned int i = 0; i < idx.size(); i++ ) {
        int length = idx[i].end - idx[i].start;
        if (length > min_length) {
            contours.push_back(std::vector<cv::Point>());
            for ( int index = idx[i].start; index < idx[i].end; index++ ) {
	      CvPoint &edge = (mEdges)[index];
                contours.back().push_back( edge);
            }
        }
    }
    return contours.size();
}

void Contour::Draw( unsigned char* pImgRGB ) {
    unsigned char *pDes;
    std::vector<cv::Range> idx = getSegmentIndexes () ;
    for ( unsigned int i = 0; i < idx.size(); i++ ) {
        unsigned char pColor[] = {rand() / ( RAND_MAX/0xFF ), rand() / ( RAND_MAX/0xFF ), rand() / ( RAND_MAX/0xFF ) };
        for ( int index = idx[i].start; index < idx[i].end; index++ ) {
            CvPoint edge = (mEdges)[index];
            pDes = pImgRGB + ( 3 * ( mImgWidth * edge.y + edge.x ) );
            for ( int j  = 0; j < 3; j++ ) {
                pDes[j] = pColor[j];
            }
        }
    }
}

void Contour::Init ( unsigned int iImgWidth, unsigned int iImgHeight, bool bAllowToModifyTheSources, unsigned char iEdgeToProcess, unsigned char iEdgeInProcess, unsigned char iEdgeProcessed, unsigned char iEdgeStrengthRemoved ) {
    if((iImgWidth != mImgWidth) || (mImgHeight != iImgHeight)  ||
            (mbAllowToModifyTheSources != bAllowToModifyTheSources) || (mEdgeToProcess != iEdgeToProcess) ||
            (mEdgeInProcess != iEdgeInProcess) || (mEdgeProcessed != iEdgeProcessed)) {
        RelaseMemory();
        mImgWidth = iImgWidth;
        mImgHeight = iImgHeight;
        mbAllowToModifyTheSources = bAllowToModifyTheSources;
        mEdgeToProcess = iEdgeToProcess;
        mEdgeInProcess = iEdgeInProcess;
        mEdgeProcessed = iEdgeProcessed;
        AllocateMemory();
    }
}

int Contour::GetImgDirectionIndex ( CvPoint tPoint ) {
    /* Direction of the change
    1   0   3
     *  *  *
      * * *
    2*******2
      * * *
     *  *  *
    3   0   1
    */
    int iIndex = 0;
    unsigned char i8BitAngle;
    float fAngle;
    double dAngle;
    switch ( mdImgEdgeDirectionType ) {
    case ANGLE_8U:
        i8BitAngle = ( ( unsigned char* ) mpImgEdgeDirection ) [tPoint.y*mImgWidth + tPoint.x];
        if ( i8BitAngle > 128 ) {
            i8BitAngle -= 128;
        }
        i8BitAngle = i8BitAngle + 16;
        iIndex = i8BitAngle / 32;
        break;
    case ANGLE_32F:
        fAngle = ( ( float* ) mpImgEdgeDirection ) [tPoint.y*mImgWidth + tPoint.x];
        if ( fAngle < 0 ) {
            fAngle += fPi;
        }
        iIndex = ( int ) ( ( fAngle + fPi/8 ) * 3/fPi );
        break;
    case ANGLE_64F:
        dAngle = ( ( double* ) mpImgEdgeDirection ) [tPoint.y*mImgWidth + tPoint.x];

        if ( dAngle < 0 ) {
            dAngle += dPi;
        }
        iIndex = ( int ) ( ( dAngle + dPi/8 ) * 3/dPi );
        break;
    }
    return iIndex;
}

void Contour::Perform ( unsigned char* pImgEdgeStrength, int defEdgeLinkMode, void* pImgEdgeDirection, int dImgEdgeDirectionType ) {
    mNrOfEdges = 0;
    mSegments.clear();
    mSegments.reserve ( 100 );
    mdefEdgeLinkMode = defEdgeLinkMode;
    mpImgEdgeDirection = pImgEdgeDirection;
    mdImgEdgeDirectionType = dImgEdgeDirectionType;
    if ( mbAllowToModifyTheSources ) {
        mpImgEdge = pImgEdgeStrength;
    } else {
        memcpy ( mpImgEdge, pImgEdgeStrength, mImgWidth * mImgHeight );
    }

    switch ( mdefEdgeLinkMode ) {
    case MODE_SIMPLE:
        Linking_Simple();
        break;
    case MODE_CONTOUR:
        Linking_Contour();
        break;
    case MODE_GRAIDENT:
        if ( pImgEdgeDirection == NULL ) {
            printf ( "The linker mode requires an edge direction image!\n" );
        }
        Linking_Gradient();
        break;
    case MODE_COMPLEX:
        if ( pImgEdgeDirection == NULL ) {
            printf ( "The linker mode requires an edge direction image!\n" );
        }
        Linking_Complex();
        break;
    }
}


void Contour::Linking_Simple() {
    CvPoint tPoint = cvPoint(0,0);
    cv::Range range(0,0);
    for ( tPoint.y = 1; tPoint.y < mImgHeight-1; tPoint.y++ ) {
        tPoint.x = 1;
        unsigned char *pCurrent = getImgEdge ( tPoint );
        for ( ; tPoint.x < mImgWidth-1; tPoint.x++ ) {
            if ( *pCurrent >= mEdgeToProcess ) {
                range.start = mNrOfEdges;
                Trace_Simple ( tPoint, &range.end );
                mSegments.push_back ( range );
                //*pCurrent = 0;
            }
            pCurrent++;
        }
    }
}

// void Contour::Trace_Simple ( CvPoint tPoint, int *pEnd ) {
//     if (isInImage(tPoint)) {
//         CvPoint tCurrent;
//         CvPoint tNext = {0,0};
//         unsigned char *pNeighbor;
//         bool bRemove = false;
//         for ( tCurrent.y = tPoint.y - 1; tCurrent.y <= tPoint.y+1; tCurrent.y++ ) {
//             tCurrent.x = tPoint.x - 1;
//             pNeighbor = getImgEdge ( tCurrent );
//             for ( ; tCurrent.x <= tPoint.x+1; tCurrent.x++ ) {
//                 if ( *pNeighbor >= mEdgeToProcess ) {
//                     if ( ( tCurrent.x == tPoint.x ) && ( tCurrent.y == tPoint.y ) ) {
//                         *pNeighbor = mEdgeProcessed;
//                         mEdges[mNrOfEdges] = tPoint;
//                         *pEnd = mNrOfEdges;
//                         mNrOfEdges++;
//                     } else {
//                         if ( bRemove ) {
//                             *pNeighbor = mEdgeProcessed;
//                         } else {
//                             bRemove = true;
//                             tNext = tCurrent;
//                         }
//                     }
//                 }
//                 pNeighbor++;
//             }
//         }
//         if ( bRemove ) {
//             Trace_Simple ( tNext, pEnd );
//         }
//     }
// }

void Contour::Trace_Simple ( CvPoint tPoint, int *pEnd ) {
    /*
      IplImage *pImgSimple = cvCreateImage(cvSize(mImgWidth, mImgHeight),8,3);
      cvZero(pImgSimple);
      cvNamedWindow ( "ImgSimple", 1 );
      for(int i = 0; i < mImgWidth * mImgHeight; i++){
        pImgSimple->imageData[i*3] = mpImgEdge[i];
      }
      cvShowImage ( "ImgSimple", pImgSimple );
      cvWaitKey(100);
      */
    if (isInImage(tPoint)) {
        unsigned char *pPix = getImgEdge ( tPoint );
        CvPoint tCurrent = tPoint;
        bool bEnd = false;
        bool bStart = false;
        std::stack<CvPoint> trace_to_start;
        std::stack<CvPoint> trace_to_end;
        std::stack<CvPoint> *pEdgeStack = &trace_to_start;
        while (bEnd == false) {
            /*
                    if(bStart == false) pImgSimple->imageData[( tCurrent.y * mImgWidth + tCurrent.x )*3+1] = 0xFF;
                    else pImgSimple->imageData[( tCurrent.y * mImgWidth + tCurrent.x )*3+2] = 0xFF;
                      cvShowImage ( "ImgSimple", pImgSimple );
                      cvWaitKey(10);
                    */
            *pPix = mEdgeProcessed;
            pEdgeStack->push(tCurrent);
            if (pPix[-mImgWidth-1] >= mEdgeToProcess) {
                pPix = pPix-mImgWidth-1;
                tCurrent.x--, tCurrent.y--;
            } else if (pPix[-mImgWidth] >= mEdgeToProcess) {
                pPix = pPix-mImgWidth;
                tCurrent.y--;
            } else if ( pPix[-mImgWidth+1] >= mEdgeToProcess) {
                pPix = pPix-mImgWidth+1;
                tCurrent.x++, tCurrent.y--;
            } else if (pPix[-1] >= mEdgeToProcess) {
                pPix = pPix-1;
                tCurrent.x--;
            } else if (pPix[+1] >= mEdgeToProcess) {
                pPix = pPix+1;
                tCurrent.x++;
            } else if (pPix[mImgWidth-1] >= mEdgeToProcess) {
                pPix = pPix + mImgWidth-1;
                tCurrent.x--, tCurrent.y++;
            } else if (pPix[mImgWidth] >= mEdgeToProcess) {
                pPix = pPix + mImgWidth;
                tCurrent.y++;
            } else if (pPix[mImgWidth+1] >= mEdgeToProcess) {
                pPix = pPix + mImgWidth + 1;
                tCurrent.x++, tCurrent.y++;
            } else {
                if (bStart == false) {
                    bStart = true;
                    while (!trace_to_start.empty()) {
                        trace_to_end.push(trace_to_start.top());
                        trace_to_start.pop();
                    }
                    pEdgeStack = &trace_to_end;
                    tCurrent = pEdgeStack->top();
                    pEdgeStack->pop();
                    pPix = getImgEdge ( tCurrent );
                } else {
                    bEnd = true;
                }
            }
        }
        while (!pEdgeStack->empty()) {
            CvPoint p = pEdgeStack->top();
            mEdges[mNrOfEdges++] = p;
            pEdgeStack->pop();
        }

        *pEnd = (mNrOfEdges-1);
    }
}

void Contour::Linking_Contour() {
    CvPoint tPoint = cvPoint(0,0);
    cv::Range range(0,0);
    for ( tPoint.y = 1; tPoint.y < ( int ) mImgHeight-1; tPoint.y++ ) {
        tPoint.x = 1;
        unsigned char *pCurrent = getImgEdge ( tPoint );
        for ( ; tPoint.x < ( int ) mImgWidth-1; tPoint.x++ ) {
            if ( *pCurrent >= mEdgeToProcess ) {
                range.start = mNrOfEdges;
                Trace_Contour ( tPoint, &range.end, 0 );
                mSegments.push_back ( range );
                //*pCurrent = 0;
            }
            pCurrent++;
        }
    }
}

void Contour::Trace_Contour ( CvPoint tPoint, int *pEnd, unsigned int iCommingFromEdge ) {
    /*
      IplImage *pImgSimple = cvCreateImage(cvSize(mImgWidth, mImgHeight),8,3);
      cvZero(pImgSimple);
      cvNamedWindow ( "ImgSimple", 1 );
      for(int i = 0; i < mImgWidth * mImgHeight; i++){
        pImgSimple->imageData[i*3] = mpImgEdge[i];
      }
      */
    if (isInImage(tPoint)) {
        unsigned char *pPix = getImgEdge ( tPoint );
        CvPoint tCurrent = tPoint;
        bool bEnd = false;
        bool bStart = false;
        std::stack<CvPoint> trace_to_start;
        std::stack<CvPoint> trace_to_end;
        std::stack<CvPoint> *pEdgeStack = &trace_to_start;
        while (bEnd == false) {
            *pPix = mEdgeProcessed;
            pEdgeStack->push(tCurrent);
            if (pPix[-mImgWidth-1] >= mEdgeToProcess) {
                pPix = pPix-mImgWidth-1;
                tCurrent.x--, tCurrent.y--;
            } else if (pPix[-mImgWidth] >= mEdgeToProcess) {
                pPix = pPix-mImgWidth;
                tCurrent.y--;
            } else if ( pPix[-mImgWidth+1] >= mEdgeToProcess) {
                pPix = pPix-mImgWidth+1;
                tCurrent.x++, tCurrent.y--;
            } else if (pPix[-1] >= mEdgeToProcess) {
                pPix = pPix-1;
                tCurrent.x--;
            } else if (pPix[+1] >= mEdgeToProcess) {
                pPix = pPix+1;
                tCurrent.x++;
            } else if (pPix[mImgWidth-1] >= mEdgeToProcess) {
                pPix = pPix + mImgWidth-1;
                tCurrent.x--, tCurrent.y++;
            } else if (pPix[mImgWidth] >= mEdgeToProcess) {
                pPix = pPix + mImgWidth;
                tCurrent.y++;
            } else if (pPix[mImgWidth+1] >= mEdgeToProcess) {
                pPix = pPix + mImgWidth + 1;
                tCurrent.x++, tCurrent.y++;
            } else {
                if (bStart == false) {
                    bStart = true;
                    while (!trace_to_start.empty()) {
                        trace_to_end.push(trace_to_start.top());
                        trace_to_start.pop();
                    }
                    pEdgeStack = &trace_to_end;
                    tCurrent = pEdgeStack->top();
                    pPix = getImgEdge ( tCurrent );
                } else {
                    bEnd = true;
                }
            }
        }
        while (!pEdgeStack->empty()) {
            mEdges[mNrOfEdges++] = pEdgeStack->top();
            pEdgeStack->pop();
        }
        *pEnd = mNrOfEdges;
    }
}


void Contour::Linking_Gradient() {
    CvPoint tPoint = cvPoint(0,0);;
    cv::Range range(0,0);
    for ( tPoint.y = 1; tPoint.y < ( int ) mImgHeight-1; tPoint.y++ ) {
        tPoint.x = 1;
        unsigned char *pCurrent = getImgEdge ( tPoint );
        for ( ; tPoint.x < ( int ) mImgWidth-1; tPoint.x++ ) {
            if ( *pCurrent >= mEdgeToProcess ) {
                range.start = mNrOfEdges;
                Trace_Gradient ( tPoint, &range.end );
                mSegments.push_back ( range );
                //*pCurrent = 0;
            }
            pCurrent++;
        }
    }
}

void Contour::Trace_Gradient ( CvPoint tPoint, int *pEnd ) {
    if (isInImage(tPoint)) {
        CvPoint tCurrent = cvPoint(0,0);;
        CvPoint tNext = cvPoint(0,0);;
        bool bRemove = false;
        unsigned char *pNeighbor;
        int iEdgeDirection;
        if ( mdImgEdgeDirectionType == ANGLE_8U) {
            mAngle8Bit[mNrOfEdges] = ( ( unsigned char* ) mpImgEdgeDirection ) [tPoint.y*mImgWidth + tPoint.x];
        }
        iEdgeDirection = GetImgDirectionIndex ( tPoint );
        int *pIndex = ( int* ) ppGradientWeightsField[iEdgeDirection];
        for ( int i = 0; i < 9; i++ ) {
            tCurrent = GetNeighborPoint ( tPoint, pIndex[i] );
            pNeighbor = getImgEdge ( tCurrent );
            if ( *pNeighbor >= mEdgeToProcess ) {
                if ( pIndex[i] == 4 ) {
                    //Center
                    *pNeighbor = mEdgeProcessed;
                    mEdges[mNrOfEdges] = tPoint;
                    *pEnd = mNrOfEdges;
                    mNrOfEdges++;
                } else {
                    if ( bRemove ) {
                        *pNeighbor = mEdgeProcessed;
                    } else {
                        bRemove = true;
                        tNext = tCurrent;
                    }
                }
            }
        }
        if ( bRemove ) {
            Trace_Gradient ( tNext, pEnd );
        }
    }
}


void Contour::Linking_Complex() {
    CvPoint tPoint = cvPoint(0,0);;
    cv::Range range(0,0);
    for ( tPoint.y = 1; tPoint.y < ( int ) mImgHeight-1; tPoint.y++ ) {
        tPoint.x = 1;
        unsigned char *pCurrent = getImgEdge ( tPoint );
        for ( ; tPoint.x < ( int ) mImgWidth-1; tPoint.x++ ) {
            if ( *pCurrent >= mEdgeToProcess ) {
                range.start = mNrOfEdges;
                Trace_Complex ( tPoint, &range.end, 0 );
                mSegments.push_back ( range );
                //*pCurrent = 0;
            }
            pCurrent++;
        }
    }
}

void Contour::Trace_Complex ( CvPoint tPoint, int *pEnd, unsigned int iCommingFromEdge ) {
    if (isInImage(tPoint)) {
        CvPoint tCurrent;
        CvPoint tNext = cvPoint(0,0);;
        bool bRemove = false;
        unsigned char *pNeighbor = NULL;
        unsigned int iGointToEdge = 0;
        int pSum[9];
        int pIndex[9];
        int iEdgeDirection;
        if ( mdImgEdgeDirectionType == ANGLE_8U) {
            mAngle8Bit[mNrOfEdges] = ( ( unsigned char* ) mpImgEdgeDirection ) [tPoint.y*mImgWidth + tPoint.x];
        }
        iEdgeDirection = GetImgDirectionIndex ( tPoint );
        SumArrayMatrix (  ( int * ) pppDirectionWeightsField[iEdgeDirection],
                          ( int * ) pppCommingFromEdgeWeightsField[iCommingFromEdge],
                          ( int * ) pSum, 9 );
        SortArrayIndexes ( pSum, pIndex, 9 );
        for ( int i = 0; i < 9; i++ ) {
            tCurrent = GetNeighborPoint ( tPoint, pIndex[i] );
            pNeighbor = getImgEdge ( tCurrent );
            if ( *pNeighbor >= mEdgeToProcess ) {
                if ( pIndex[i] == 4 ) {
                    //Center
                    *pNeighbor = mEdgeProcessed;
                    mEdges[mNrOfEdges] = tPoint;
                    *pEnd = mNrOfEdges;
                    mNrOfEdges++;
                } else {
                    if ( bRemove ) {
                        *pNeighbor = mEdgeProcessed;
                    } else {
                        bRemove = true;
                        tNext = tCurrent;
                        iGointToEdge = 8-pIndex[i];
                    }
                }
            }
        }
        if ( bRemove ) {
            Trace_Complex ( tNext, pEnd, iGointToEdge );
        }
    }
}

const CvPoint Contour::GetNeighborPoint ( CvPoint tPoint, int iNeighborIndex ) {
    switch ( iNeighborIndex ) {
    case 0:
        tPoint.x += -1;
        tPoint.y += -1;
        break;
    case 1:
        tPoint.x +=  0;
        tPoint.y += -1;
        break;
    case 2:
        tPoint.x += +1;
        tPoint.y += -1;
        break;
    case 3:
        tPoint.x += -1;
        tPoint.y +=  0;
        break;
    case 4:
        tPoint.x +=  0;
        tPoint.y +=  0;
        break;
    case 5:
        tPoint.x += +1;
        tPoint.y +=  0;
        break;
    case 6:
        tPoint.x += -1;
        tPoint.y += +1;
        break;
    case 7:
        tPoint.x +=  0;
        tPoint.y += +1;
        break;
    case 8:
        tPoint.x += +1;
        tPoint.y += +1;
        break;
    }
    return tPoint;
}
int Contour::GetEdgeListSplittedXY (std::vector<cv::Point_<int> > &rEdges, std::vector<unsigned char> **ppAngle8Bit) {

    rEdges.resize ( mEdges.size() );
    for (unsigned int i = 0; i < mEdges.size(); i++) {
        rEdges[i] = mEdges[i];
    }
    if (ppAngle8Bit != NULL) {
        *ppAngle8Bit = &mAngle8Bit;
    }
    return mEdges.size();
}

const void Contour::SortArrayIndexes ( int *pArray, int *pIndexes, const int iSize ) {
    int iMax = -1;
    int iMin = iMax;
    int iMaxIndex = 0;
    for ( int j = 0; j < iSize; j++ ) {
        for ( int i = 0; i < iSize; i++ ) {
            if ( pArray[i] > iMax ) {
                iMax = pArray[i];
                iMaxIndex = i;
            }
            if ( pArray[i] < iMin ) {
                iMin = pArray[i];
            }
        }
        pIndexes[j] = iMaxIndex;
        pArray[iMaxIndex] = iMin;
        iMax = iMin;
    }
}

void  Contour::GetAbnormitiesInEdgesImage ( IplImage *ptImgEdge, std::vector<CvPoint> *pAbnormities, uchar iEdgeStrength ) {
    pAbnormities->clear();
    bool bAbnormities;
    CvPoint tPoint;
    for ( tPoint.y = 1; tPoint.y < ptImgEdge->height-1; tPoint.y++ ) {
        unsigned char *pRow0 = ( unsigned char * ) ptImgEdge->imageData + ptImgEdge->width* ( tPoint.y-1 );
        unsigned char *pRow1 = pRow0 + ptImgEdge->width;
        unsigned char *pRow2 = pRow1 + ptImgEdge->width;
        for ( tPoint.x = 1; tPoint.x < ptImgEdge->width-1; tPoint.x++ ) {
            bAbnormities = false;
            if ( pRow1[1] > iEdgeStrength ) {
                int iNoOfNeighbors = 0;
                if ( pRow0[0] > iEdgeStrength ) {
                    iNoOfNeighbors++;
                    if ( ( pRow0[1] > iEdgeStrength ) || ( pRow1[0] > iEdgeStrength ) ) {
                        bAbnormities = true;
                    }
                }
                if ( pRow0[1] > iEdgeStrength ) {
                    iNoOfNeighbors++;
                    if ( ( pRow0[0] > iEdgeStrength ) || ( pRow0[2] > iEdgeStrength ) ) {
                        bAbnormities = true;
                    }
                }
                if ( pRow0[2] > iEdgeStrength ) {
                    iNoOfNeighbors++;
                    if ( ( pRow0[1] > iEdgeStrength ) || ( pRow1[2] > iEdgeStrength ) ) {
                        bAbnormities = true;
                    }
                }
                if ( pRow1[0] > iEdgeStrength ) {
                    iNoOfNeighbors++;
                    if ( ( pRow0[0] > iEdgeStrength ) || ( pRow2[0] > iEdgeStrength ) ) {
                        bAbnormities = true;
                    }
                }
                if ( pRow1[2] > iEdgeStrength ) {
                    iNoOfNeighbors++;
                    if ( ( pRow0[2] > iEdgeStrength ) || ( pRow2[2] > iEdgeStrength ) ) {
                        bAbnormities = true;
                    }
                }
                if ( pRow2[0] > iEdgeStrength ) {
                    iNoOfNeighbors++;
                    if ( ( pRow1[0] > iEdgeStrength ) || ( pRow2[1] > iEdgeStrength ) ) {
                        bAbnormities = true;
                    }
                }
                if ( pRow2[1] > iEdgeStrength ) {
                    iNoOfNeighbors++;
                    if ( ( pRow2[0] > iEdgeStrength ) || ( pRow2[2] > iEdgeStrength ) ) {
                        bAbnormities = true;
                    }
                }
                if ( pRow2[2] > iEdgeStrength ) {
                    iNoOfNeighbors++;
                    if ( ( pRow2[1] > iEdgeStrength ) || ( pRow1[2] > iEdgeStrength ) ) {
                        bAbnormities = true;
                    }
                }
                if ( bAbnormities ) {
                    pAbnormities->push_back ( tPoint );
                }
            }
            pRow0++;
            pRow1++;
            pRow2++;
        }
    }

}

std::vector<cv::Range> Contour::getSegmentIndexes() {
    return mSegments;
}

} //namespace V4R
