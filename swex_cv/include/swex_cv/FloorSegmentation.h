#ifndef FLOORSEGMENTATION_H
#define FLOORSEGMENTATION_H

#include <opencv2/opencv.hpp>
#include <swex_cv/SwexMath.h>
#include <swex_cv/Matrix.h>

#define BOTTOM_DEVIATION 30
#define STRUCTURE_DEVIATION 10



using namespace sm;

inline void LOG(const std::string & s) { std::cout << s << std::endl; }

class FloorSegmentation
{
public:
    FloorSegmentation();
    FloorSegmentation(int nCannyThresh, int nCannyThresh2, int nHoughLinesThresh, int nMinHorLength, int nMinVerLength, int nHorSlopeThresh, int nVerSlopeThresh);
    FloorSegmentation(const FloorSegmentation & floorFinder);
    virtual ~FloorSegmentation();

    const FloorSegmentation&    operator=(const FloorSegmentation & floorFinder);
    void                        initFrom(const FloorSegmentation & floorFinder);

    void                        setBaseValue(int nCannyThresh = 50, int nCannyThresh2 = 150, int nHoughLinesThresh = 80, int nMinHorLength = 15, int nMinVerLength = 60, int nHorSlopeThresh = 45, int nVerSlopeThresh = 5);

    void                        setCannyThreh(int nThresh)          { m_nCannyThresh        = nThresh;      }
    void                        setCannyThreh2(int nThresh2)        { m_nCannyThresh2       = nThresh2;     }
    void                        setHoughLinesThresh(int nThresh)    { m_nHoughLinesThresh   = nThresh;      }
    void                        setMinHorLength(int nMinLength)     { m_nMinHorLength       = nMinLength;   }
    void                        setMinVerLength(int nMinLength)     { m_nMinVerLength       = nMinLength;   }
    void                        setHorSlopeThresh(int nThresh)      { m_nHorSlopeThresh     = nThresh;      }
    void                        setVerSlopeThresh(int nThresh)      { m_nVerSlopeThresh     = nThresh;      }

    int                         cannyThresh()                       { return m_nCannyThresh;                }
    int                         cannyThresh2()                      { return m_nCannyThresh2;               }
    int                         houghLinesThresh()                  { return m_nHoughLinesThresh;           }
    int                         threshThresh()                      { return m_nThreshThresh;               }
    int                         minHorLength()                      { return m_nMinHorLength;               }
    int                         minVerLength()                      { return m_nMinVerLength;               }
    int                         horSlopeThresh()                    { return m_nHorSlopeThresh;             }
    int                         verSlopeThresh()                    { return m_nVerSlopeThresh;             }

    double                      findFloor(const cv::Mat & srcMat, Polygon &out);

private:
    Matrix<double>              m_SvmMatrix;

    cv::Mat                     m_SrcMat;
    cv::Mat                     m_CannyMat;
    cv::Mat                     m_ThreshMat;

    std::vector<Line>           m_vLines;
    std::vector<Line>           m_vHorLines;
    std::vector<Line>           m_vVerLines;

    std::vector<double>         m_vGlobalScore;
    std::vector<double>         m_vStructScore;
    std::vector<double>         m_vBottomScore;
    std::vector<double>         m_vHomogeScore;

    Polyline                    m_BottomPoly;

    int                         m_nCannyThresh;
    int                         m_nCannyThresh2;

    int                         m_nHoughLinesThresh;

    int                         m_nThreshThresh;

    int                         m_nMinHorLength;
    int                         m_nMinVerLength;

    int                         m_nHorSlopeThresh;
    int                         m_nVerSlopeThresh;

    void                        distributeLines();
    void                        clearVerLine();
    void                        clearHorLine();

    void                        buildFloor(Polygon &out);

    double                      computeBottomScore();
    double                      computeStructScore();
    double                      computeHomogeScore();
    double                      computeGlobalScore();
}; // FloorFinder

#endif // FLOORSEGMENTATION_H
