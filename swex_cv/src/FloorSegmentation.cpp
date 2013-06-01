#include <swex_cv/FloorSegmentation.h>
#include <swex_cv/ColorSegmentation.h>

FloorSegmentation::FloorSegmentation()
{
    setBaseValue();
}

FloorSegmentation::FloorSegmentation(int nCannyThresh, int nCannyThresh2, int nHoughLinesThresh, int nMinHorLength, int nMinVerLength, int nHorSlopeThresh, int nVerSlopeThresh)
{
    setBaseValue(nCannyThresh, nCannyThresh2, nHoughLinesThresh, nMinHorLength, nMinVerLength, nHorSlopeThresh, nVerSlopeThresh);
}

FloorSegmentation::FloorSegmentation(const FloorSegmentation & floorFinder)
{
    initFrom(floorFinder);
}

FloorSegmentation::~FloorSegmentation()
{
    ;// todo
}

const FloorSegmentation & FloorSegmentation::operator=(const FloorSegmentation & floorFinder)
{
    initFrom(floorFinder);
    return *this;
}

void FloorSegmentation::initFrom(const FloorSegmentation & floorFinder)
{
    m_SvmMatrix = Matrix<double>(3, 3);
    m_SvmMatrix.set(0, 0, 3.5);
    m_SvmMatrix.set(0, 1, 26);
    m_SvmMatrix.set(0, 2, 475);
    m_SvmMatrix.set(1, 0, 26);
    m_SvmMatrix.set(1, 1, 50);
    m_SvmMatrix.set(1, 2, -125);
    m_SvmMatrix.set(2, 0, 475);
    m_SvmMatrix.set(2, 1, -125);
    m_SvmMatrix.set(2, 2, 10);

    setBaseValue(floorFinder.m_nCannyThresh,
                 floorFinder.m_nCannyThresh2,
                 floorFinder.m_nHoughLinesThresh,
                 floorFinder.m_nMinHorLength,
                 floorFinder.m_nMinVerLength,
                 floorFinder.m_nHorSlopeThresh,
                 floorFinder.m_nVerSlopeThresh);
}

void FloorSegmentation::setBaseValue(int nCannyThresh, int nCannyThresh2, int nHoughLinesThresh, int nMinHorLength, int nMinVerLength, int nHorSlopeThresh, int nVerSlopeThresh)
{
    m_nCannyThresh          = nCannyThresh;
    m_nCannyThresh2         = nCannyThresh2;
    m_nHoughLinesThresh     = nHoughLinesThresh;
    m_nMinHorLength         = nMinHorLength;
    m_nMinVerLength         = nMinVerLength;
    m_nHorSlopeThresh       = nHorSlopeThresh;
    m_nVerSlopeThresh       = nVerSlopeThresh;
    m_nThreshThresh         = 0;
}

double FloorSegmentation::findFloor(const cv::Mat & srcMat, Polygon & out)
{
    int nRet = 0;
    m_SrcMat = srcMat;
    m_vHorLines.clear();
    m_vVerLines.clear();

    distributeLines();
    clearVerLine();
    clearHorLine();

    computeBottomScore();
    computeStructScore();
    computeHomogeScore();
    computeGlobalScore();


    for (uint i = 0; i < m_vHorLines.size(); i++)
        cv::line(m_SrcMat, cv::Point(m_vHorLines[i][0], m_vHorLines[i][1]), cv::Point(m_vHorLines[i][2], m_vHorLines[i][3]), cv::Scalar(255, 0, 0), 2);
    for (uint i = 0; i < m_vVerLines.size(); i++)
        cv::line(m_SrcMat, cv::Point(m_vVerLines[i][0], m_vVerLines[i][1]), cv::Point(m_vVerLines[i][2], m_vVerLines[i][3]), cv::Scalar(0, 255, 0), 2);

    buildFloor(out);

    if (out.size() > 1)
    {
        cv::line(m_SrcMat, cv::Point(0, out.front().y), out.front(), cv::Scalar(0, 0, 255), 2);
        for (uint i = 1; i < out.size(); i++)
            cv::line(m_SrcMat, out[i-1], out[i], cv::Scalar(0, 0, 255), 2);
        cv::line(m_SrcMat, out.back(), cv::Point(m_SrcMat.cols, out.back().y), cv::Scalar(0, 0, 255), 2);
    }

    return average(m_vGlobalScore);
}

void FloorSegmentation::distributeLines()
{
    double                  tmp_slope;

    cv::Canny(m_SrcMat, m_CannyMat, m_nCannyThresh, m_nCannyThresh2);
    cv::HoughLinesP(m_CannyMat, m_vLines, 1, CV_PI / 180, ((m_nHoughLinesThresh > 0) ? m_nHoughLinesThresh : 1), 10, 10);

    for (uint i = 0; i < m_vLines.size(); i++)
    {
        double x            = (m_vLines[i][3] - m_vLines[i][1]);
        double y            = (m_vLines[i][2] - m_vLines[i][0]);
        double line_length  = sqrt(square(x) + square(y));
        tmp_slope           = abs(getSlope(x, y));

        if ((tmp_slope < (90 + m_nHorSlopeThresh)  && tmp_slope > (90 - m_nHorSlopeThresh))
          && (line_length > m_nMinHorLength))
            m_vHorLines.push_back(m_vLines[i]);
        else if ((tmp_slope < (180 + m_nVerSlopeThresh) && tmp_slope > (180 - m_nVerSlopeThresh))
              &&  line_length > m_nMinVerLength)
            m_vVerLines.push_back(m_vLines[i]);
    }
}

void FloorSegmentation::clearVerLine()
{
    int yMid = m_SrcMat.rows / 4;
    std::vector<Line> tmpVerLines;
    for (uint i = 0; i < m_vVerLines.size(); i++)
    {
        if(m_vVerLines[i][1] > yMid)
            tmpVerLines.push_back(m_vVerLines[i]);
    }

    m_vVerLines = tmpVerLines;
}

void FloorSegmentation::clearHorLine()
{
    std::vector<Line>       tmpHorLines;
    std::vector<cv::Point>  intersections;
    cv::Rect                img(0, 0, m_SrcMat.cols, m_SrcMat.rows);
    double                  yVanish = 0.;
    double                  ySum = 0.;
    int                     count = 0;

    for (uint i = 1; i < m_vHorLines.size(); i++)
    {
        uint j = i-1;
        // on trouve l'equation des droites i et i-1 de la forme : ax + by + c = 0
        int ai  = (m_vHorLines[i][1] - m_vHorLines[i][3]),
            bi  = (m_vHorLines[i][2] - m_vHorLines[i][0]),
            ci  = ((m_vHorLines[i][0] * m_vHorLines[i][3]) - (m_vHorLines[i][2] * m_vHorLines[i][1])),
            aj  = (m_vHorLines[j][1] - m_vHorLines[j][3]),
            bj  = (m_vHorLines[j][2] - m_vHorLines[j][0]),
            cj  = ((m_vHorLines[j][0] * m_vHorLines[j][3]) - (m_vHorLines[j][2] * m_vHorLines[j][1])),
            wx  = (bj * ci) - (cj * bi),
            wy  = (cj * ai) - (aj * ci),
            w   = (aj * bi) - (bj * ai);
            w   = (w != 0) ? w : 1;
        intersections.push_back(cv::Point(wx / w, wy / w));
    }

    // on calcul la moyenne des intersections à l'intérieur de l'image
    for (uint i = 0; i < intersections.size(); i ++)
    {
        if (img.contains(intersections[i]))
        {
            ySum += intersections[i].y;
            count ++;
        }
    }

    yVanish = ySum / count;

    for (uint i = 0; i < m_vHorLines.size(); i++)
        if ((m_vHorLines[i][1] >= yVanish) && (m_vHorLines[i][3] >= yVanish)) tmpHorLines.push_back(m_vHorLines[i]);

    m_vHorLines = tmpHorLines;
}

void FloorSegmentation::buildFloor(Polygon & out)
{
    for (uint i = 0; i < m_vHorLines.size(); i++)
    {
        if (m_vGlobalScore[i] >= 0.5)
        {
            out.push_back(cv::Point(m_vHorLines[i][0], m_vHorLines[i][1]));
            out.push_back(cv::Point(m_vHorLines[i][2], m_vHorLines[i][3]));
        }
    }
    std::sort(out.begin(), out.end(), pointXComparator);
}

double FloorSegmentation::computeBottomScore()
{
    uint        i       = 0;
    Polyline    poly;
    m_vBottomScore.clear();
    m_vBottomScore.resize(m_vHorLines.size(), 0);

    std::sort(m_vVerLines.begin(), m_vVerLines.end(), lineFirstXComparator);

    if (m_vVerLines.size())
        poly.push_back(cv::Point(0, m_vVerLines[0][1]));

    for (; i < m_vVerLines.size(); i++)
        poly.push_back(cv::Point(m_vVerLines[i][1], m_vVerLines[i][1]));

    if(m_vVerLines.size())
        poly.push_back(cv::Point(m_SrcMat.cols, m_vVerLines[i][1]));

    for (uint i = 0; i < m_vHorLines.size(); i++)
    {
        Line        l       = m_vHorLines[i];
        cv::Vec2i   eq      = getLineEq(l);
        double      weight  = 0;

        for (int x = l[0]; x <= l[2]; x++)
            weight += dPoint2Poly(cv::Point(x, applyStraightEq(x, eq)), poly);
        m_vBottomScore[i] = exp(-(weight / (2 * pow(BOTTOM_DEVIATION, 2))));
    }

    return average(m_vBottomScore);
}

double FloorSegmentation::computeStructScore()
{
    m_vStructScore.clear();
    m_vStructScore.resize(m_vHorLines.size(), 0);

    return average(m_vStructScore);
}

double FloorSegmentation::computeHomogeScore()
{
    ColorSegmentation   cs;
    std::vector<Cloud>  clouds;
    Cloud*              pCloud = NULL;

    m_vHomogeScore.clear();
    m_vHomogeScore.resize(m_vHorLines.size(), 0);
    cs.segmentColor(m_SrcMat, clouds);

    for (uint i = 0; i < clouds.size(); i++)
    {
        if (pCloud && (clouds.at(i).size() > pCloud->size()))
            pCloud = &clouds[i];
        else
            pCloud = &clouds[i];
    }

    if (pCloud)
    {
        for (uint i = 0; i < m_vHorLines.size(); i++)
        {
            cv::Vec2i eq = getLineEq(m_vHorLines[i]);
            double count = 0;

            for (int x = m_vHorLines[i][0]; x <= m_vHorLines[i][2]; x++)
            {
                for (uint j = 0; j < pCloud->size(); j++)
                {
                    if ((*pCloud)[j].x == x && (*pCloud)[j].y > applyStraightEq(x, eq))
                        count++;
                }
            }
            m_vHomogeScore[i] = count / pCloud->size();
        }
    }

    return average(m_vHomogeScore);
}

double FloorSegmentation::computeGlobalScore()
{
    m_vGlobalScore.clear();
    m_vGlobalScore.resize(m_vHorLines.size(), 0.);

    for (uint i = 0; i < m_vHorLines.size(); i++)
        m_vGlobalScore[i] = (m_vStructScore[i] * 1.6) + (m_vBottomScore[i] * 0.75) + m_vHomogeScore[i];

    return average(m_vGlobalScore);
}
