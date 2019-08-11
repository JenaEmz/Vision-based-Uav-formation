#include <string>
#include <time.h>
#include <fstream>  

//#include <nav_msgs/Odometry.h>

using namespace std;
class DataRecorder
{
private:
    int m_id;
    string m_name;
    string m_file_name;
    fstream m_file;
public:
    DataRecorder(int id,string name);
    void Record(const string& msg);
    ~DataRecorder();
};

DataRecorder::DataRecorder(int id,string name):m_id(id),m_name(name)
{
    char timeinfo[256] ={0};
    time_t nowtime = time(NULL);
    struct tm *p;
    p = gmtime(&nowtime);

    sprintf(timeinfo,"./%d_%d_%d_%d_%02d_",1900+p->tm_year,1+p->tm_mon,p->tm_mday,8+p->tm_hour,p->tm_min);
    m_file_name = string(timeinfo);
    m_file_name+=name;
    m_file_name+=".txt";
    m_file.open(m_file_name.c_str());
    m_file<<"timestamp\ttarget_x\ttarget_y\ttarget_z\tx\ty\tz\tyaw\tinit_state\ttrans_data\n";
    m_file.close();
}

DataRecorder::~DataRecorder()
{
}
void DataRecorder::Record(const string& msg)
{
    m_file.open(m_file_name.c_str());
    m_file<<msg.c_str()<<"\n";
    m_file.close();
}
/*DataRecorder::GroundTruthCallback(const nav_msgs::Odometry msg)
{

}*/