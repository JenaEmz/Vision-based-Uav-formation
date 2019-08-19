#include "DataRecorder.h"


DataRecorder::DataRecorder(void):m_id(0),m_name("")
{

}
DataRecorder::DataRecorder(int id,string name):m_id(id),m_name(name)
{
    char timeinfo[256] ={0};
    time_t nowtime = time(NULL);
    struct tm *p;
    p = gmtime(&nowtime);

    sprintf(timeinfo,"/home/jena/csq_ws/logs/%d_%d_%d_%d_%02d_",1900+p->tm_year,1+p->tm_mon,p->tm_mday,8+p->tm_hour,p->tm_min);
    m_file_name = string(timeinfo);
    m_file_name+=name;
    m_file_name+=".txt";
    m_file_fd = open(m_file_name.c_str(),ios::out|ios::in);
    char head[]="timestamp\tinit_state\tx\ty\tz\ttrans_data\ttrans_data_raw\tgroundtruth_x\tgroundtruth_y\tgroundtruth_z\n";
    write(m_file_fd,head,strlen(head));
}

DataRecorder::~DataRecorder()
{
    close(m_file_fd);
}
void DataRecorder::Record(char data[],int size)
{
    write(m_file_fd,data,strlen(data));
}