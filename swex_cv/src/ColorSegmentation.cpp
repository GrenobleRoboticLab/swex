#include <swex_cv/ColorSegmentation.h>
#include <set>

#define THRESHOLD(size,c) (c/size)

Universe::Universe(int elements)
{
    elts = new UniElt[elements];
    num = elements;
    for (int i = 0; i < elements; i++)
    {
        elts[i].rank = 0;
        elts[i].size = 1;
        elts[i].p = i;
    }
}

Universe::~Universe()
{
    delete [] elts;
}

int Universe::find(int x)
{
    int y = x;
    while (y != elts[y].p)
        y = elts[y].p;
    elts[x].p = y;
    return y;
}

void Universe::join(int x, int y)
{
    if (elts[x].rank > elts[y].rank)
    {
        elts[y].p = x;
        elts[x].size += elts[y].size;
    } else {
        elts[x].p = y;
        elts[y].size += elts[x].size;
        if (elts[x].rank == elts[y].rank)
            elts[y].rank++;
    }
    num--;
}

ColorSegmentation::ColorSegmentation()
{ setAll();m_pUniverse = NULL; }

ColorSegmentation::ColorSegmentation(double sigma, double k, int minSize)
{ setAll(sigma, k, minSize);m_pUniverse = NULL; }

ColorSegmentation::ColorSegmentation(const ColorSegmentation &cs)
{ initFrom(cs);m_pUniverse = NULL; }

ColorSegmentation::~ColorSegmentation() { releaseUniverse(); }

const ColorSegmentation& ColorSegmentation::operator =(const ColorSegmentation& cs)
{
    initFrom(cs);
    return *this;
}

void ColorSegmentation::initFrom(const ColorSegmentation &cs)
{ setAll(cs.m_dSigma, cs.m_dK, cs.m_nMinSize); }

void ColorSegmentation::setAll(double sigma, double k, int minSize)
{
    m_dSigma    = sigma;
    m_dK        = k;
    m_nMinSize  = minSize;
}

void ColorSegmentation::segmentColor(const cv::Mat &src, std::vector<Cloud> &outClouds)
{
    m_vSplitedImg.resize(src.channels());
    cv::split(src, m_vSplitedImg);
    m_nWidth    = src.cols;
    m_nHeight   = src.rows;

    applyGaussianBlur();
    buildEdges();
    segmentGraph();

    genClouds(outClouds);
}

void ColorSegmentation::applyGaussianBlur()
{
    for (uint i = 0; i < m_vSplitedImg.size(); i++)
        cv::GaussianBlur(m_vSplitedImg[i], m_vSplitedImg[i], cv::Size(25, 25), m_dSigma);
}

void ColorSegmentation::buildEdges()
{
    uint count = 0;
    m_vEdges.resize(m_nWidth * m_nHeight * 4);

    for (uint x = 0; x < m_nWidth; x++)
    {
        for (uint y = 0; y < m_nHeight; y++)
        {
            if (x < (m_nWidth - 1))
            {
                m_vEdges[count].a = y * m_nWidth + x;
                m_vEdges[count].b = y * m_nWidth + (x + 1);
                m_vEdges[count].w = diff(x, y, (x + 1), y);
                count++;
            }

            if (y < (m_nHeight - 1))
            {
                m_vEdges[count].a = y * m_nWidth + x;
                m_vEdges[count].b = (y + 1) * m_nWidth + x;
                m_vEdges[count].w = diff(x, y, x, (y + 1));
                count++;
            }

            if ((x < (m_nWidth - 1)) && (y < (m_nHeight - 1))) {
                m_vEdges[count].a = y * m_nWidth + x;
                m_vEdges[count].b = (y+1) * m_nWidth + (x+1);
                m_vEdges[count].w = diff(x, y, x+1, y+1);
                count++;
            }

            if ((x < m_nWidth-1) && (y > 0))
            {
                m_vEdges[count].a = y * m_nWidth + x;
                m_vEdges[count].b = (y-1) * m_nWidth + (x+1);
                m_vEdges[count].w = diff(x, y, x+1, y-1);
                count++;
            }
        }
    }
}

void ColorSegmentation::segmentGraph()
{
    uint num_vertices = m_nWidth * m_nHeight;
    // sort edges by weight
    std::sort(m_vEdges.begin(), m_vEdges.end(), compEdge);

    releaseUniverse();

    // make a disjoint-set forest
    m_pUniverse = new Universe(num_vertices);

    // init thresholds
    float *threshold = new float[num_vertices];
    for (uint i = 0; i < num_vertices; i++)
      threshold[i] = THRESHOLD(1,m_dK);

    // for each edge, in non-decreasing weight order...
    for (uint i = 0; i < m_vEdges.size(); i++)
    {
      Edge *pedge = &m_vEdges[i];

      // components conected by this edge
      int a = m_pUniverse->find(pedge->a);
      int b = m_pUniverse->find(pedge->b);
      if (a != b) {
        if ((pedge->w <= threshold[a]) &&
            (pedge->w <= threshold[b])) {
          m_pUniverse->join(a, b);
          a = m_pUniverse->find(a);
          threshold[a] = pedge->w + THRESHOLD(m_pUniverse->size(a), m_dK);
        }
      }
    }

    for (uint i = 0; i < m_vEdges.size(); i++) {
        int a = m_pUniverse->find(m_vEdges[i].a);
        int b = m_pUniverse->find(m_vEdges[i].b);

        if ((a != b) && ((m_pUniverse->size(a) < m_nMinSize) || (m_pUniverse->size(b) < m_nMinSize)))
            m_pUniverse->join(a, b);
    }

    // free up
    delete threshold;
}

double ColorSegmentation::diff(uint x1, uint y1, uint x2, uint y2)
{
    double dV = 0.;

    for (uint i = 0; i < m_vSplitedImg.size(); i ++)
        dV += square((float)*m_vSplitedImg[i].ptr(y1, x1) - *m_vSplitedImg[i].ptr(y2, x2));

    return sqrt(dV);
}

void ColorSegmentation::genClouds(std::vector<Cloud> & outPolys)
{
    typedef std::set<CloudPoly, CompCloudPoly> SCloud;

    SCloud cloud;

    outPolys.clear();

    for (uint y = 0; y < m_nHeight; y++)
    {
        for (uint x = 0; x < m_nWidth; x++)
        {
            int             comp = m_pUniverse->find(y * m_nWidth + x);
            bool            found = false;

            SCloud::iterator it = cloud.begin();

            for (; it != cloud.end(); ++it)
            {
                if ((*it).comp == comp)
                {
                    found = true;
                    CloudPoly p = *it;
                    p.points.push_back(cv::Point(x, y));
                    cloud.insert(p);
                }

            }

            if (!found)
                cloud.insert(CloudPoly(comp, cv::Point(x, y)));
        }
    }
}

void ColorSegmentation::releaseUniverse()
{
    if (m_pUniverse)
        delete m_pUniverse;
    m_pUniverse = NULL;
}
