#ifndef COLORSEGMENTATION_H
#define COLORSEGMENTATION_H

#include <opencv2/opencv.hpp>
#include <swex_cv/SwexMath.h>

using namespace sm;
namespace color_segmentation {

template<typename T>
inline void swap(std::vector<T> & v, int dst, int src)
{
    if (dst >= 0 && dst < v.size() && src >= 0 && src < v.size())
    {
        T temp = v.at(dst);
        v[dst] = v.at(src);
        v[src] = temp;
    }
}

inline double iangle(std::vector<cv::Point> & vPs, int dst, int src)
{
    return getSlope(vPs[dst].x - vPs[src].x, vPs[dst].y - vPs[src].y);
}

struct Edge {
    Edge() { w = 0.f; a = b = 0; }
    float   w;
    int     a,
            b;
};

inline bool compEdge(const Edge & e1, const Edge & e2) { return (e1.w < e2.w); }

struct CloudPoly {
    CloudPoly() { ; }
    CloudPoly(int c, const cv::Point & point) { comp = c; points.push_back(point); }

    std::vector<cv::Point>  points;
    int                     comp;

    // void appendPoint(const cv::Point & point) { points.push_back(point); }
};

struct CompCloudPoly { bool operator()(const CloudPoly & poly1, const CloudPoly & poly2) { return (poly1.comp < poly2.comp); } };

struct UniElt {
  UniElt() { rank = p = size = 0; }
  int   rank;
  int   p;
  int   size;
};

class Universe {
public:
    Universe(int elements = 0);
    ~Universe();
    int     find(int x);
    void    join(int x, int y);
    int     size(int x) const { return elts[x].size; }
    int     num_sets() const { return num; }

private:
    UniElt* elts;
    int     num;
}; // class Universe

} // namespace color_segmentation

using namespace color_segmentation;

class ColorSegmentation {
public:
    ColorSegmentation();
    ColorSegmentation(double sigma, double k, int minSize);
    ColorSegmentation(const ColorSegmentation & cs);

    virtual ~ColorSegmentation();

    const ColorSegmentation&    operator=(const ColorSegmentation & cs);
    void                        initFrom(const ColorSegmentation & cs);

    void                        setAll(double sigma = 2.5, double k = 5000, int minSize = 100);

    void                        segmentColor(const cv::Mat & src, std::vector<Cloud> & outClouds);

    void                        setSigma(double sigma)  { m_dSigma      = sigma;    }
    void                        setK(double k)          { m_dK          = k;        }
    void                        setMinSize(int minSize) { m_nMinSize    = minSize;  }

    double                      getSigma()              { return m_dSigma;          }
    double                      getK()                  { return m_dK;              }
    int                         getMinSize()            { return m_nMinSize;        }

private:
    double                      m_dSigma;
    double                      m_dK;
    int                         m_nMinSize;

    uint                        m_nWidth;
    uint                        m_nHeight;

    Universe*                   m_pUniverse;

    std::vector<cv::Mat>        m_vSplitedImg;
    std::vector<Edge>           m_vEdges;

    void                        applyGaussianBlur();
    void                        buildEdges();
    void                        segmentGraph();
    double                      diff(uint x1, uint y1, uint x2, uint y2);
    void                        genClouds(std::vector<Cloud> &outPolys);

    void                        releaseUniverse();
};

#endif // COLORSEGMENTATION_H
