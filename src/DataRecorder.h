#include <string>
#include <time.h>
#include <fstream>  
#include <unistd.h>
#include <stdio.h>
#include <cstring>
#include <fcntl.h>
#include <dirent.h>

#include <sys/stat.h> 
//#include <nav_msgs/Odometry.h>

using namespace std;
class DataRecorder
{
private:
    int m_id;
    string m_name;
    string m_file_name;
    
public:
    int m_file_fd;
    DataRecorder(const char* head,const char* path);
    DataRecorder(void);
    void Record(char data[],int size);
    ~DataRecorder();
};

/*DataRecorder::GroundTruthCallback(const nav_msgs::Odometry msg)
{

}*/