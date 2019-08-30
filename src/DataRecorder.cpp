#include "DataRecorder.h"


DataRecorder::DataRecorder(void):m_id(0),m_name("")
{

}
DataRecorder::DataRecorder(const char* head,const char* path)
{
    mode_t fileperms;

    int openflags = O_WRONLY | O_CREAT | O_TRUNC;

    //rwxrw-r-x
    fileperms = S_IRUSR | S_IWUSR | S_IXUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IXOTH;
    m_file_fd = open(path,openflags, fileperms);
    printf("log name %s\n",path);
    if(m_file_fd<0)
    {
        printf("open log error\n");
    }
    //char head[256]={0};
    //sprintf(head,"%dtimestamp\t%dinit_state\t%dx\t%dy\t%dz\t%dtrans_data\t%dtrans_data_raw\t%dgroundtruth_x\t%dgroundtruth_y\t%dgroundtruth_z\n",id,id,id,id,id,id,id,id,id,id);
    
    if(write(m_file_fd,head,strlen(head))<0)
    {
        printf("write log error\n");
    }
}

DataRecorder::~DataRecorder()
{
    close(m_file_fd);
}
void DataRecorder::Record(char data[],int size)
{
    write(m_file_fd,data,strlen(data));
}