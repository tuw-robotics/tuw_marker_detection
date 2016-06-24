/**
 * @file contour
 * @author Markus Bader
 * @date Mo Jul 9 2007
 * @version 0.1
 * @brief Includes the header file for a class to link detected image edges
 *
 * @see
 **/
#ifndef CONTOUR_H
#define CONTOUR_H

#include <vector>
#include <valarray>

#include <opencv2/core/core.hpp>

namespace tuw {


/**
* @brief Class to link detected images edges a typical source can be a cvCanny image
**/
class Contour {
public:

    Contour();
    ~Contour();
    /**
    * @brief Static const variable used to define the source format of a gradient image as atan2(dy, dx) of the edge
    * The format ANGLE_8U means zero rad are 0, +PI and -PI are 0x7F
    **/
    static const int ANGLE_8U;
    /**
    * @brief Static const variable used to define the source format of a gradient image as atan2(dy, dx) of the edge
    * The format ANGLE_32F means values from -PI to +PI
    **/
    static const int ANGLE_32F;
    /**
    * @brief Static const variable used to define the source format of a gradient image as atan2(dy, dx) of the edge
    * The format ANGLE_64F means values from -PI to +PI
    **/
    static const int ANGLE_64F;

    /**
    * @brief Static const variable used to define the methode used to find the contour (linked edges)
    **/
    static const int MODE_SIMPLE;
    /**
    * @brief Static const variable used to define the methode used to find the contour (linked edges)
    **/
    static const int MODE_CONTOUR;
    /**
    * @brief Static const variable used to define the methode used to find the contour (linked edges)
    **/
    static const int MODE_GRAIDENT;
    /**
    * @brief Static const variable used to define the methode used to find the contour (linked edges)
    **/
    static const int MODE_COMPLEX;



    /**
    * @brief Inititalized the class and reserves the momory needed
    * @param iImgWidth image width
    * @param iImgHeight image hight
    * @param bAllowToModifyTheSources if it is true it will write into your source image based on the parameters iEdgeStrengthSrc and iEdgeStrengthDes, it it it false it will make a copy of the image first (solwer)
    * @param iEdgeToProcess the strength of an edge in the source image, only of interrest if bAllowToModifyTheSources = true
    * @param iEdgeInProcess the strength of an edge in process, only of interrest if bAllowToModifyTheSources = true
    * @param iEdgeProcessed the strength of an edge after it was processed, only of interrest if bAllowToModifyTheSources = true
    * @param iEdgeStrengthRemoved the strength of an edge whenn it was removed, only of interrest if bAllowToModifyTheSources = true
    */
    void Init ( unsigned int iImgWidth, unsigned int iImgHeight, bool bAllowToModifyTheSources = false, unsigned char iEdgeToProcess = 0xFF, unsigned char iEdgeInProcess = 0xFF-1, unsigned char iEdgeProcessed = 0xFF-2, unsigned char iEdgeStrengthRemoved = 0 );
    /**
    * @brief Starts the edge linking with a certain mode
    * @param pImgEdgeStrength edge image, where the edge must be at least of the iEdgeStrengthSrc defined in Contour::Init
    * @param defEdgeLinkMode defines the used mode to link the edges the default is MODE_CONTOUR </br>
    * MODE_SIMPLE the linker follows the first detected neighbor and removes the others. It checks first the top left and goes than to the right and then the next row (pImgEdgeDirection and defImgEdgeDirectionType are not needed)</br>
    * MODE_CONTOUR the linker follows the first detected neighbor detected on the obosite site from which it steped to the current edge and removes the others. (pImgEdgeDirection and defImgEdgeDirectionType are not needed)</br>
    * MODE_GRAIDENT the linker follows the first detected neighbor detected given by the edge direction in the pImgEdgeDirection image.  (pImgEdgeDirection is needed and defImgEdgeDirectionType only if the format of pImgEdgeDirection is unlike ANGLE_8U)</br>
    * MODE_COMPLEX mixes the modes MODE_GRAIDENT and MODE_CONTOUR </br>
    * @param pImgEdgeDirection pointer to the edge direction image with the angle information of every edge in the pImgEdgeStrength
    * @param defImgEdgeDirectionType type of the pImgEdgeDirection default is ANGLE_8U.</br>
    * ANGLE_8U the angle values in pImgEdgeDirection are encoded as 8 bit values where zero rad are 0, +PI and -PI are 0x7F.</br>  "unsigned char i8BitAngle = (unsigned char) (dRad * 255.0 / (2 * CV_PI) + 128.5);" is a way to generate such information. </br>
    * ANGLE_32F the angle values in pImgEdgeDirection are encoded 32 float value where the ange is defined as atan2(dy, dx) of the related edge in pImgEdgeStrength </br>
    * ANGLE_64F the angle values in pImgEdgeDirection are encoded 32 float value where the ange is defined as atan2(dy, dx) of the related edge in pImgEdgeStrength </br>
    **/
    void Perform ( unsigned char* pImgEdgeStrength, int defEdgeLinkMode = MODE_CONTOUR, void* pImgEdgeDirection = NULL, int defImgEdgeDirectionType = ANGLE_8U );

    /**
    * @brief Draws the detected contours in a RGB image with radom colours
    * @param pImgRGB target image
    **/
    void Draw( unsigned char* pImgRGB );

    /**
    * @brief Returns a number of edges used to linke edges to semgents
    * @return number of edges used to linke edges to semgents
    * @see Contour::GetEdgeList
    **/
    unsigned int GetNrOfEdges() {
        return mNrOfEdges;
    };

    /**
    * @brief Returns number of edges and edges in the arguments
    * @param listX
    * @param listY
    * @return number of edges used to linke edges to semgents
    **/
    int GetEdgeListSplittedXY (std::vector<cv::Point_<int> > &rEdges, std::vector<unsigned char> **ppAngle8Bit = NULL);
    
    /**
    * @brief Returns the contour as vector of vector points
    * @param contours
    * @param min_length on zero all controus are returned
    * @return number of contours
    **/
    int getContours( std::vector<std::vector<cv::Point> > &contours, unsigned min_length = 0);

    /**
    * @brief the indexes of the contour segments
    * @return indexes
    **/
    std::vector<cv::Range> getSegmentIndexes();
    
    /**
    * @brief Find egde contour Abnormities in edge image
    * @return a vetor with points of the position abnormities
    **/
    void GetAbnormitiesInEdgesImage ( IplImage *ptImgEdge, std::vector<CvPoint> *pAbnormities, uchar iEdgeStrength = 0 );

private:
    unsigned char *mpImgEdge;
    void *mpImgEdgeDirection;
    int mdImgEdgeDirectionType;
    int mdefEdgeLinkMode;
    int mImgWidth;
    int mImgHeight;
    bool mbAllowToModifyTheSources;
    unsigned char mEdgeToProcess;
    unsigned char mEdgeInProcess;
    unsigned char mEdgeProcessed;
    static const int pppDirectionWeightsField[4][3][3];
    static const int ppGradientWeightsField[4][9];
    static const int pppCommingFromEdgeWeightsField[9][3][3];
    static const int ppContourWeightsField[9][9];
    static const float fPi;
    static const double dPi;
    //CvPoint *mpEdges;
    std::vector<CvPoint> mEdges;
    std::vector<unsigned char> mAngle8Bit;
    unsigned int mNrOfEdges;
    std::vector<cv::Range> mSegments;

    void AllocateMemory();
    void RelaseMemory();
    void Linking_Simple();
    int GetImgDirectionIndex ( CvPoint tPoint );
    void Trace_Simple ( CvPoint tPoint, int *pEnd );
    void Linking_Complex();
    void Trace_Complex ( CvPoint tPoint, int *pEnd, unsigned int iCommingFromEdge );
    void Linking_Contour();
    void Trace_Contour ( CvPoint tPoint, int *pEnd, unsigned int iCommingFromEdge );
    void Linking_Gradient();
    void Trace_Gradient ( CvPoint tPoint, int *pEnd );

    static const CvPoint GetNeighborPoint ( CvPoint pPtrCenter, int iNeighborIndex );
    static const void SortArrayIndexes ( int *pArray, int *pIndexes, const int iSize );

//Inlines
    inline unsigned char* getImgEdge ( CvPoint tPoint ) {
        return mpImgEdge + ( tPoint.y * mImgWidth + tPoint.x );
    };
    inline void SumArrayMatrix ( int *pMatrixA, int *pMatrixB, int *pSum, const int iSize ) {
        for ( int i = 0; i < iSize; i++ ) pSum[i] = pMatrixA[i] + pMatrixB[i];
    };
    inline bool isInImage( const CvPoint &p){
        return ( ( p.x > 0 ) && ( p.x < mImgWidth-1 ) && ( p.y > 0 ) && ( p.y < mImgHeight-1 ) );
    }
    inline void markNeighborEdgesAsProcessed(unsigned char *pPix){
        pPix[ mImgWidth-1] = mEdgeProcessed;
        pPix[ mImgWidth  ] = mEdgeProcessed;
        pPix[ mImgWidth+1] = mEdgeProcessed;
        pPix[-1] = mEdgeProcessed;
        pPix[ 1] = mEdgeProcessed;
        pPix[-mImgWidth-1] = mEdgeProcessed;
        pPix[-mImgWidth  ] = mEdgeProcessed;
        pPix[-mImgWidth+1] = mEdgeProcessed;
    }
};

}

#endif //CONTOUR_H
// kate: indent-mode cstyle; space-indent on; indent-width 4; 
