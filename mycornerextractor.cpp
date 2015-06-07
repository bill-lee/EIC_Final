#include "mycornerextractor.h"

lab405::MyCornerExtractor::MyCornerExtractor()
{

}

lab405::MyCornerExtractor::~MyCornerExtractor()
{

}

void lab405::MyCornerExtractor::GetCornerFeature(std::vector<double> _range_data)
{
//    std::ifstream infile(filename);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//    int temp;
//    infile >> temp;
    std::size_t range;
    double resolution = CV_PI/360;

    for (int i = 0; i < 361; i++)
    {
//        infile >> range;
        range = _range_data.at(i);
        if (/*i > 0 &&*/ range < 5000 /*&& abs(pre - range < 500)*/)
        {
            double angle = i*resolution;
            pcl::PointXYZRGB point;
            point.x = static_cast<double>(range)*cos(angle)/100;
            point.y = static_cast<double>(range)*sin(angle)/100;
            point.z = 0;


            uint8_t r = 255;
            uint8_t g = 255;
            uint8_t b = 255;
            // pack r/g/b into rgb
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            point.rgb = *reinterpret_cast<float*>(&rgb);
            cloud->points.push_back(point);
        }
    }
    cloud->points.push_back(pcl::PointXYZRGB(0, 255, 0));


    std::size_t windows = 11;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _show(new pcl::PointCloud<pcl::PointXYZRGB>(*cloud));
    for(int i = 0; i < cloud->points.size() - windows; i++)
    {

//        if (CornerCheckXYZRGB(cloud, windows, i, cv::Point2d(-0.3436, 0.3515), 0.15, 0.1, cv::Point2d(0.5, 0.5)))
        if (CornerCheckXYZRGB(cloud, windows, i, cv::Point2d(-0.5, 0.5), 0.3, 0.3, cv::Point2d(1, 1)))
        {
            std::cout << i + windows/2 << std::endl;



            pcl::PointXYZRGB point;
//            point.x = cloud->points.at(i).x;
//            point.y = cloud->points.at(i).y;
//            point.z = 0;


            uint8_t r = 255;
            uint8_t g = 0;
            uint8_t b = 0;
            // pack r/g/b into rgb
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            point.rgb = *reinterpret_cast<float*>(&rgb);
//            _show->points.push_back(point);

//            point.x = cloud->points.at(i + windows/2).x;
//            point.y = cloud->points.at(i + windows/2).y;
//            point.z = 0;
//            _show->points.push_back(point);
//            cloud->points.push_back(point);


//            point.x = cloud->points.at(i + windows - 1).x;
//            point.y = cloud->points.at(i + windows - 1).y;
//            point.z = 0;
//            _show->points.push_back(point);

            for (int j = 0; j < windows - 1; j++)
            {
                point.x = cloud->points.at(i + j).x;
                point.y = cloud->points.at(i + j).y;
                point.z = 0;
                _show->points.push_back(point);
            }





        }



    }
//    pcl::visualization::CloudViewer viewer("Simple Viewer");
//    viewer.showCloud(_show);
//    while (!viewer.wasStopped())
//    {
//    }

//    pcl::visualization::CloudViewer viewer("Simple Viewer");
//    viewer.showCloud(cloud);
//    while (!viewer.wasStopped())
//    {
//    }

}

bool lab405::MyCornerExtractor::CornerCheckXYZRGB(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, std::size_t windows, std::size_t start, cv::Point2d f_tor, double d_tor, double c_tor, cv::Point2d dis_var_tor)
{
    std::size_t half_len = windows/2;

    cv::Point2d vi (cloud->points.at(start + half_len).x - cloud->points.at(start).x,
                    cloud->points.at(start + half_len).y - cloud->points.at(start).y);
    cv::Point2d vj (cloud->points.at(start + half_len).x - cloud->points.at(start + windows - 1).x,
                    cloud->points.at(start + half_len).y - cloud->points.at(start + windows - 1).y);

    double c2 =  (cloud->points.at(start).x - cloud->points.at(start + windows - 1).x)*(cloud->points.at(start).x - cloud->points.at(start + windows - 1).x)
            + (cloud->points.at(start).y - cloud->points.at(start + windows - 1).y)*(cloud->points.at(start).y - cloud->points.at(start + windows - 1).y);
    double a2 = vi.x*vi.x + vi.y*vi.y;
    double b2 = vj.x*vj.x + vj.y*vj.y;
    double f = (c2 - (a2 + b2))/(2*sqrt(a2)*sqrt(b2));
//    std::cout << "f = " << f << std::endl;
    if (f < f_tor.x || f > f_tor.y)
        return false;

    cv::Point2d dvi (cloud->points.at(start + half_len + 1).x - cloud->points.at(start).x,
                    cloud->points.at(start + half_len + 1).y - cloud->points.at(start).y);
    cv::Point2d dvj (cloud->points.at(start + half_len + 1).x - cloud->points.at(start + windows - 1).x,
                    cloud->points.at(start + half_len + 1).y - cloud->points.at(start + windows - 1).y);

    double da2 = dvi.x*dvi.x + dvi.y*dvi.y;
    double db2 = dvj.x*dvj.x + dvj.y*dvj.y;
    double df = (c2 - (da2 + db2))/(2*sqrt(da2)*sqrt(db2));
    if (abs(df - f) > d_tor)
        return false;

    cv::Point2d dvi2 (cloud->points.at(start + half_len - 1).x - cloud->points.at(start).x,
                    cloud->points.at(start + half_len - 1).y - cloud->points.at(start).y);
    cv::Point2d dvj2 (cloud->points.at(start + half_len - 1).x - cloud->points.at(start + windows - 1).x,
                    cloud->points.at(start + half_len - 1).y - cloud->points.at(start + windows - 1).y);

    double dda2 = dvi2.x*dvi2.x + dvi2.y*dvi2.y;
    double ddb2 = dvj2.x*dvj2.x + dvj2.y*dvj2.y;
    double ddf = (c2 - (dda2 + ddb2))/(2*sqrt(dda2)*sqrt(ddb2));
    if (abs(ddf - f) > d_tor)
        return false;

    std::vector<double> dis_i;
    std::vector<double> dis_j;
    for (int i = 0; i < half_len - 1; i++)
    {
        cv::Point2d vic (cloud->points.at(start + i + 1).x - cloud->points.at(start + i).x,
                         cloud->points.at(start + i + 1).y - cloud->points.at(start + i).y);
        cv::Point2d vjc (cloud->points.at(start + windows - i - 2).x - cloud->points.at(start + windows - i - 1 ).x,
                         cloud->points.at(start + windows - i - 2).y - cloud->points.at(start + windows - i - 1 ).y);

        double dis_vic = vic.x*vic.x + vic.y*vic.y;
        std::cout << "vic = " << sqrt(dis_vic) << ", tor = " << 1 - (vic.x * vi.x + vic.y * vi.y)/(sqrt(dis_vic)*sqrt(a2));
        dis_i.push_back(sqrt(dis_vic));
        if (1 - (vic.x * vi.x + vic.y * vi.y)/(sqrt(dis_vic)*sqrt(a2)) > c_tor)
            return false;

        double dis_vjc = vjc.x*vjc.x + vjc.y*vjc.y;
        std::cout << ", vjc = " << sqrt(dis_vjc) << ", tor = " << 1 - (vjc.x * vj.x + vjc.y * vj.y)/(sqrt(dis_vjc)*sqrt(b2)) << std::endl;
        dis_j.push_back(sqrt(dis_vjc));
        if (1 - (vjc.x * vj.x + vjc.y * vj.y)/(sqrt(dis_vjc)*sqrt(b2)) > c_tor)
            return false;


    }

    double m_dis_i = 0.0;
    double m_dis_j = 0.0;
    int count = 1;
    for (int i = 0; i < dis_i.size(); i++)
    {
        m_dis_i = m_dis_i + (dis_i.at(i) - m_dis_i)/count;
        count++;
    }

    count = 1;
    for (int i = 0; i < dis_j.size(); i++)
    {
        m_dis_j = m_dis_j + (dis_j.at(i) - m_dis_j)/count;
        count++;
    }



    double acc = 0.0;
    for (int i = 0; i < dis_i.size(); i++)
        acc += (dis_i.at(i) - m_dis_i)*(dis_i.at(i) - m_dis_i);
    double stdv_dis_i = sqrt(acc/dis_i.size());
    double cv_i = stdv_dis_i/m_dis_i;
    std::cout << "stdv_dis_i = " << stdv_dis_i << ", m_dis_i = " << m_dis_i << ", cv_i = " << cv_i << std::endl;
    if (cv_i > dis_var_tor.x)
        return false;

    acc = 0.0;
    for (int i = 0; i < dis_j.size(); i++)
        acc += (dis_j.at(i) - m_dis_j)*(dis_j.at(i) - m_dis_j);
    double stdv_dis_j = sqrt(acc/dis_j.size());
    double cv_j = stdv_dis_j/m_dis_j;
    std::cout << "stdv_dis_j = " << stdv_dis_j << ", m_dis_j = " << m_dis_j << ", cv_j = " << cv_j << std::endl;
    if (cv_j > dis_var_tor.y)
        return false;

    return true;
}
