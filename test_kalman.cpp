#include "fast_kalman_filter.h"
#include <arpa/inet.h>
#include <assert.h>
#include <cassert>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <netinet/in.h>
#include <pthread.h>
#include <queue>
#include <sched.h>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/shm.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <wiringPi.h>
#include <typeinfo>

extern "C" {
	#include "spi_if.h"
	#include "icm20948.h"
}

#define SERV_PORT 11206
// #define SERV_IP "192.168.9.107"
// #define SERV_IP "192.168.8.12"
// #define SERV_IP "192.168.1.15"
// #define SERV_IP "192.168.7.3"
#define SERV_IP "192.168.7.6"
// #define SERV_IP "192.168.11.16"
#define BUFF_SIZE 19
#define ENABLE_INTERRUPUT	0
#define ICM_INT_EVENT     0x01
#define NEW_FILE_EVENT    0x03
#define FILE_CLOSE_EVENT  0x04
#define LOG_START_EVENT   0x05
#define LOG_STOP_EVENT    0x06
#define APP_EXIT_EVENT    0x07
#define SENORS_NUMBER     12

using namespace std;

std::queue<char> icm_queue;
ofstream icm_logfile;
string icm_filename;
int cs[SENORS_NUMBER] = {
	SENSOR_ID_1,
	SENSOR_ID_2,
	SENSOR_ID_9,
	SENSOR_ID_10,
	SENSOR_ID_11,
	SENSOR_ID_6,
	SENSOR_ID_7,
	SENSOR_ID_8,
	SENSOR_ID_3,
	SENSOR_ID_4,
	SENSOR_ID_5,
	1
};
int is_sensor = 1;
int ai_s_s[11][10];
int ai_s_g[10];
double ad_t1[11][2];
double ad_time[10];
double ad_offset_gyr[11][3] = {
-1.69123, 0.0362687, 0.0841869, // 0
-1.31059, 0.65105, -0.0561486, // 1
0, 0, 0, // 2
-2.31251, -0.981632, -0.0700461, // 3
-0.087015, 1.02642, -0.340759, // 4
-0.415738, 0.907245, 0.224542, // 5
0.338392, 0.273673, -0.143446, // 6
-0.823387, -0.251574, 0.378052, // 7
-0.969793, -0.919281, 0.336813, // 8
-1.0424, 1.34548, -0.551292, // 9
-0.0881989, 0.101813, 0.15864  // 10
};


/*

double ad_offset_gyr[11][3] = {
-1.67587, 0.0200962, 0.0634778, // 0
-1.30008, 0.695442, -0.0101105, // 1
0, 0, 0, // 2
-2.37881, -1.05414, -0.12868, // 3
-0.211922, 0.87763, -0.414642, // 4
-0.524944, 0.817079, 0.226286, // 5
0.466108, 0.284916, -0.0790574, // 6
-0.823387, -0.251574, 0.378052, // 7
-0.940674, -0.917588, 0.310257, // 8
-1.0672, 1.33514, -0.514733, // 9
-0.0881989, 0.101813, 0.15864  // 10
};


-1.31059, 0.65105, -0.0561486, // 1


double ad_offset_gyr[11][3] = {
-1.7004, 0.0278228, 0.0763007, // 0
-2.02696, 1.1988, 0.0156933, // 1
0, 0, 0, // 2
-2.28011, -1.02882, -0.122548, // 3
0.0786054, 0.977083, -0.387658, // 4
-0.3985, 0.887159, 0.264964, // 5
0.312495, 0.275749, -0.0945285, // 6
-0.796058, -0.262845, 0.377394, // 7
-0.970482, -0.932081, 0.339732, // 8
-1.0283, 1.34631, -0.512014, // 9
-0.171187, 0.132835, 0.168834  // 10
};

*/

#if ENABLE_INTERRUPUT
void icm_handler(void)
{
	icm_queue.push(ICM_INT_EVENT);
}
#endif
int sock_client_fd;
void *icm_log_thread(void *threadid)
{
	size_t len[SENORS_NUMBER];
	for(int i=0; i<SENORS_NUMBER;i++)
	{
		len[i] = 0;
	}
	fast_kalman_filter *fkf = (fast_kalman_filter*)malloc(sizeof(fast_kalman_filter)*SENORS_NUMBER);
	sensor_data data[SENORS_NUMBER];
	double roll_truth, pitch_truth, yaw_truth;
	double roll_rmse=0.0, pitch_rmse=0.0, yaw_rmse=0.0;
	double ax[SENORS_NUMBER],ay[SENORS_NUMBER],az[SENORS_NUMBER];
	double wx[SENORS_NUMBER],wy[SENORS_NUMBER],wz[SENORS_NUMBER];
	double mx[SENORS_NUMBER],my[SENORS_NUMBER],mz[SENORS_NUMBER];
	double angle_x[SENORS_NUMBER],angle_y[SENORS_NUMBER],angle_z[SENORS_NUMBER];
	double temp[SENORS_NUMBER];
	for(int i=0; i<SENORS_NUMBER;i++)
	{
		sensor_object_init(&data[i]);
		fast_kalman_filter_init(&fkf[i]);
		data[i].deltaT = 1.0/103.0;
	}
	long tid;
	float ax_sensor[SENORS_NUMBER],ay_sensor[SENORS_NUMBER],az_sensor[SENORS_NUMBER];
	float gx_sensor[SENORS_NUMBER],gy_sensor[SENORS_NUMBER],gz_sensor[SENORS_NUMBER];
	float mx_sensor[SENORS_NUMBER],my_sensor[SENORS_NUMBER],mz_sensor[SENORS_NUMBER];
	float temp_sensor[SENORS_NUMBER];
	stringstream out;
	struct timeval tp;
	tid = (long)threadid;
	
	//ax=ay=az=gx=gy=gz=mx=my=mz=temp=0;
	
	
	for(int i = 0; i < 11; i++)
	{
		printf("\n\nInitialize sensor %d:\n",i);
		ai_s_s[i][0] = 1;
		sensor_select(cs[i]);
		configMPU();
		configAK09916();
		ai_s_s[i][0] = is_sensor; // 1:true
		is_sensor = 1;
	}
	sock_client_fd = socket(AF_INET, SOCK_STREAM, 0);
	
	struct sockaddr_in server_addr;
	memset(&server_addr, 0, sizeof(0));
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(SERV_PORT);
	server_addr.sin_addr.s_addr = inet_addr(SERV_IP);
	while(1)
	{
#if !ENABLE_INTERRUPUT
		if(checkDataReady())
		{
			icm_queue.push(ICM_INT_EVENT);
			//printf("data ready 1\r\n");
		}
#endif
		if(!icm_queue.empty())
		{
			switch(icm_queue.front())
			{
				case ICM_INT_EVENT:
				gettimeofday(&tp,NULL);

				for(int i=0; i<11; i++)
				{
					if(!ai_s_s[i][0]) continue;
					sensor_select(cs[i]);
					readSensor(&ax_sensor[i],&ay_sensor[i],&az_sensor[i],&gx_sensor[i],&gy_sensor[i],&gz_sensor[i],&mx_sensor[i],&my_sensor[i],&mz_sensor[i],&temp_sensor[i]);
					//mx_sensor[i] = 1.0;
					//my_sensor[i] = 0.0;
					//mz_sensor[i] = 0.0;
					/*
					out<<tp.tv_sec<<"."<<tp.tv_usec<<",";
					out<<gx<<","<<gy<<","<<gz<<",";
					out<<ax<<","<<ay<<","<<az<<",";
					out<<mx<<","<<my<<","<<mz<<",";
					out<<temp;
					out<<endl;
					cout<<"["<<i<<"],";
					cout<<tp.tv_sec<<"."<<tp.tv_usec<<",";
					cout<<gx_sensor[i]<<","<<gy_sensor[i]<<","<<gz_sensor[i]<<",";
					cout<<ax_sensor[i]<<","<<ay_sensor[i]<<","<<az_sensor[i]<<",";
					cout<<mx_sensor[i]<<","<<my_sensor[i]<<","<<mz_sensor[i]<<",";
					cout<<temp_sensor[i];
					cout<<endl;
					*/
					// cout<<tp.tv_sec<<"."<<tp.tv_usec<<",";
					ax[i] = (double)ax_sensor[i];
					ay[i] = (double)ay_sensor[i];
					az[i] = (double)az_sensor[i];
					wx[i] = (double)gx_sensor[i]*M_PI/180.0;
					wy[i] = (double)gy_sensor[i]*M_PI/180.0;
					wz[i] = (double)gz_sensor[i]*M_PI/180.0;
					mx[i] = (double)mx_sensor[i];
					my[i] = (double)my_sensor[i];
					mz[i] = (double)mz_sensor[i];
					temp[i] = temp_sensor[i];
					angle_x[i] = angle_x[i] + data[i].deltaT * gx_sensor[i] - data[i].deltaT * ad_offset_gyr[i][0];
					angle_y[i] = angle_y[i] + data[i].deltaT * gy_sensor[i] - data[i].deltaT * ad_offset_gyr[i][1];
					angle_z[i] = angle_z[i] + data[i].deltaT * gz_sensor[i] - data[i].deltaT * ad_offset_gyr[i][2];
					/*
					cout<<"["<<i<<"],";
					cout<<tp.tv_sec<<"."<<tp.tv_usec<<",";
					cout<<wx[i]<<","<<wy[i]<<","<<wz[i]<<",";
					cout<<ax[i]<<","<<ay[i]<<","<<az[i]<<",";
					cout<<mx[i]<<","<<my[i]<<","<<mz[i]<<",";
					cout<<temp[i];
					cout<<endl;
					*/
					len[i]++;
					data[i].fAccel[X] = -ax[i];
					data[i].fAccel[Y] = ay[i];
					data[i].fAccel[Z] = -az[i];

					data[i].fGyro[X] = -wx[i];
					data[i].fGyro[Y] = wy[i];
					data[i].fGyro[Z] = -wz[i];

					data[i].fMag[X] = -mx[i];
					data[i].fMag[Y] = my[i];
					data[i].fMag[Z] = -mz[i];

					fast_kalman_filter_update(&fkf[i],&data[i]);
					quat2euler_f64(&(fkf[i].q_k), &(fkf[i].euler));

					if (ai_s_g[2] == 0)
					{
						if(1){
							printf("\n sensor error: ");
							for(int i2 = 0; i2 < 11; i2++)
							{
								if(ai_s_s[i2][0] || i2 == 2) continue;
								printf("[%2d] ", i2);
							}
							printf("\n");
						}
						ai_s_g[2] = 1;
					}

					if(ai_s_g[0] == 0) ai_s_g[0] = 0;
					double d_tcp_time = (double)tp.tv_usec / 1000000 + tp.tv_sec;

					if (ai_s_g[0] == 0)
					{
						if(ai_s_s[i][1] == 0)
						{
							ad_t1[i][0] = (double)tp.tv_usec / 1000000 + tp.tv_sec;
							ai_s_s[i][1]++;
						}
						double _dtime0 = 2.0;
						double _dtime1 = 4.0;
						double _dtime2 = 0.1;
						if(ai_s_s[i][1] == 1 && d_tcp_time - ad_t1[i][0] > _dtime0) {
							// cout << i << " offset start" << endl;
							ad_t1[i][0] = (double)tp.tv_usec / 1000000 + tp.tv_sec;
							angle_x[i] = angle_y[i] = angle_z[i] = 0;
							ai_s_s[i][1]++;
						}
						if(ai_s_s[i][1] == 2 && d_tcp_time - ad_t1[i][0] > _dtime1){
							// cout << i << " offset: ";
							ad_t1[i][1] = (double)tp.tv_usec / 1000000 + tp.tv_sec - ad_t1[i][0];
							ad_offset_gyr[i][0] += angle_x[i] / ad_t1[i][1];
							ad_offset_gyr[i][1] += angle_y[i] / ad_t1[i][1];
							ad_offset_gyr[i][2] += angle_z[i] / ad_t1[i][1];
							// for (int j = 0; j < 3; ++j) cout << ad_offset_gyr[i][j] << " ";
							// cout << endl;
							ai_s_s[i][1]++;
						}

						if(ai_s_s[i][1] == 3){
							// cout << i << " offset check" << endl;
							angle_x[i] = angle_y[i] = angle_z[i] = 0;
							ad_t1[i][0] = (double)tp.tv_usec / 1000000 + tp.tv_sec;
							angle_x[i] = angle_y[i] = angle_z[i] = 0;
							ai_s_s[i][1]++;
						}

						if(ai_s_s[i][1] == 5 && ai_s_s[i][2] && ai_s_g[0] == 0){
							ai_s_g[0] = 1;
							cout << "double ad_offset_gyr[11][3] = {" << endl;
							int _i = 0;
							for (; _i < 10; ++_i){
								for (int _j = 0; _j < 3; ++_j) cout << ad_offset_gyr[_i][_j] << ", ";
									cout << "// " << _i << endl;
							}
							for (int _j = 0; _j < 2; ++_j) cout << ad_offset_gyr[_i][_j] << ", ";
								cout << ad_offset_gyr[_i][2];
							cout << "  // " << _i << endl;
							cout << "};" << endl;
						}
						if(ai_s_s[i][1] == 4 && d_tcp_time - ad_t1[i][0] > _dtime2) {
							// printf("%d offset end %f: %f, %f, %f\n", i, (double)tp.tv_usec / 1000000 + tp.tv_sec - ad_t1[i][0] , angle_x[i], angle_y[i], angle_z[i]);
							angle_x[i] = angle_y[i] = angle_z[i] = 0;
							ai_s_s[i][1]++;
							ai_s_s[i][2] = 1;
						}
					}


					if(ai_s_g[0] == 1){
						printf("\nwait tcp...\n");
						int i_sock;
						while(1){
							i_sock = connect(sock_client_fd, (struct sockaddr*)&server_addr, sizeof(server_addr));
							if (i_sock < 0) {
								perror("tcp error");
								// exit(1);
							} else break;
						}
						printf("\nconnected\n");
						ai_s_g[0]++;
					}
					if(ai_s_g[0]){
						float tcp_w;
						float tcp_x;
						float tcp_y;
						float tcp_z;
						if (ai_s_g[2] == 1)
						{
							ad_time[0] = (double)tp.tv_usec / 1000000 + tp.tv_sec;
							ai_s_g[2] = 2;
						}
						int _index = 0;
						char _c1 = 'h';
						char _c2 = 't';
						char sendbuff[BUFF_SIZE];
						memcpy(sendbuff + _index, &_c1, 1);
						memcpy(sendbuff + BUFF_SIZE - 1, &_c2, 1);
						_index += 1;

						char id_char = (char)i + 11;
						if(id_char < 11 || id_char > 22){
							cout << "ID error: " << id_char << endl;
							continue;
						}
						memcpy(sendbuff + _index, &id_char, 1);
						_index += 1; // 2

						if (i == 0) {

							// tcp_x = (float)data[i].fAccel[X];
							// tcp_y = (float)data[i].fAccel[Y];
							// tcp_z = (float)data[i].fAccel[Z];
							tcp_x = ax_sensor[i];
							tcp_y = ay_sensor[i];
							tcp_z = az_sensor[i];
							// if(i == 0) printf("[%2d]: %0.2f, %0.2f, %0.2f\n",i , tcp_x, tcp_y, tcp_z);
							memcpy(sendbuff + _index, &tcp_x, 4);
							_index += 4;
							memcpy(sendbuff + _index, &tcp_y, 4);
							_index += 4;
							memcpy(sendbuff + _index, &tcp_z, 4);
							_index += 4;
							// 14

							tcp_x = (float)((double)tp.tv_usec / 1000000 + tp.tv_sec - ad_time[0]);
							// printf("%f\n", tcp_x);
							memcpy(sendbuff + _index, &tcp_x, 4);

						}
						else {
							tcp_x = (float)angle_x[i];
							tcp_y = (float)angle_y[i];
							tcp_z = (float)angle_z[i];
							// if(i == 1) printf("[%2d]: %0.2f, %0.2f, %0.2f\n",i , tcp_x, tcp_y, tcp_z);
							memcpy(sendbuff + _index, &tcp_x, 4);
							_index += 4;
							memcpy(sendbuff + _index, &tcp_y, 4);
							_index += 4;
							memcpy(sendbuff + _index, &tcp_z, 4);
							_index += 4;
							// 14

							// if (1)
							if (i == 1)
							{
								ai_s_s[i][2]++;
								if (ai_s_s[i][2] % 30 == 0)
								{
									printf("[%2d]: %.4f, %.4f, %.4f\n",i , tcp_x, tcp_y, tcp_z);
									// printf("%2d\n", i);
									// printf("%f\n", tcp_x);
								}
							}

						}
						_index += 4;
						// 18
/*

						tcp_x = (float)fkf[i].q_k.x;
						tcp_y = (float)fkf[i].q_k.y;
						tcp_z = (float)fkf[i].q_k.z;
						tcp_w = (float)fkf[i].q_k.w;
						memcpy(sendbuff + _index, &tcp_x, 4);
						_index += 4;
						memcpy(sendbuff + _index, &tcp_y, 4);
						_index += 4;
						memcpy(sendbuff + _index, &tcp_z, 4);
						_index += 4;
						memcpy(sendbuff + _index, &tcp_w, 4);
						_index += 4;
						// 34
						*/
						/*
						tcp_x = (float)fkf[i].euler.pitch;
						tcp_y = (float)fkf[i].euler.yaw;
						tcp_z = (float)fkf[i].euler.roll;
						// printf("[%2d]: %0.2f, %0.2f, %0.2f\n",i, tcp_x, tcp_y, tcp_z);
						memcpy(sendbuff + _index, &tcp_x, 4);
						_index += 4;
						memcpy(sendbuff + _index, &tcp_y, 4);
						_index += 4;
						memcpy(sendbuff + _index, &tcp_z, 4);
						_index += 4;
						// 30

						*/

						_index ++;
						// +1


						// BUFF_SIZE, 19
						// BUFF_SIZE, 35
						if(_index == BUFF_SIZE && ai_s_g[0] == 2){
							send(sock_client_fd, sendbuff, BUFF_SIZE, 0);
						}

					}

					if(icm_logfile.is_open())
					{
					/*
					icm_logfile<<"["<<i<<"],";
					icm_logfile<<tp.tv_sec<<"."<<tp.tv_usec<<",";
					icm_logfile<<wx[i]<<","<<wy[i]<<","<<wz[i]<<",";
					icm_logfile<<ax[i]<<","<<ay[i]<<","<<az[i]<<",";
					icm_logfile<<mx[i]<<","<<my[i]<<","<<mz[i]<<",";
					icm_logfile<<temp[i];
					icm_logfile<<endl;
					*/
					//icm_logfile<<tp.tv_sec<<"."<<tp.tv_usec<<" ";
						icm_logfile<<wx[i]<<" "<<wy[i]<<" "<<wz[i]<<" ";
						icm_logfile<<ax[i]<<" "<<ay[i]<<" "<<az[i]<<" ";
						icm_logfile<<mx[i]<<" "<<my[i]<<" "<<mz[i]<<" ";
						icm_logfile<<temp[i];
						icm_logfile<<endl;
					}

				}
				break;
				case NEW_FILE_EVENT:
				printf("\r\n[INFO] create new ICM log file.\r\n");
				out.clear();
				if(icm_logfile.is_open())
					icm_logfile.close();
				icm_logfile.open(icm_filename.c_str());
				assert(!icm_logfile.fail());
			//write headers
				if(icm_logfile.is_open())
				{
				/*icm_logfile<<"seconds.milliseconds,";
				icm_logfile<<"gx(dps),gy,gz,";
				icm_logfile<<"ax(g),ay,az,";
				icm_logfile<<"mx(uT),my,mz,";
				icm_logfile<<"temperature(degree),";
				icm_logfile<<endl;
				*/
				}
				break;
				case FILE_CLOSE_EVENT:
				if(icm_logfile.is_open())
				{
				//icm_logfile<<out.str();
					icm_logfile.close();
					out.clear();
				}
				printf("\r\n[INFO] close ICM log file.\r\n");
				break;
				case LOG_START_EVENT:
				printf("\r\n[INFO] ICM log start.\r\n");	
				break;
				case LOG_STOP_EVENT:
				printf("\r\n[INFO] ICM log stop.\r\n");	
				break;
				case APP_EXIT_EVENT:
				if(icm_logfile.is_open())
				{
				//icm_logfile<<out.str();
					icm_logfile.close();
					out.clear();
				}
				printf("\r\n[INFO] exit ICM thread.\r\n");
				pthread_exit(NULL);
				break;
				default:

				break;
			}
			icm_queue.pop();
		}
	}
}


int main()
{
	char key = 0;
	pthread_t icm_thread;
	pthread_attr_t tattr;
	sched_param param;
	
	spi_init();

#if ENABLE_INTERRUPUT
	if(wiringPiISR(ICM_INTERRUPT_PIN,INT_EDGE_RISING,&icm_handler) < 0)
	{
		printf("[ERROR] Unable to setup ICM ISR.\r\n");
	}
#endif
	
	pthread_attr_init(&tattr);
	pthread_attr_getschedparam (&tattr, &param);
	param.sched_priority = 99;
	pthread_attr_setschedparam (&tattr, &param);
	pthread_create(&icm_thread,&tattr,icm_log_thread,(void *)NULL);

	while(1)
	{
		cout<<endl;
		cout<<"1: create new file for logging data"<<endl;
		cout<<"2: close and save data file"<<endl;
		cout<<"3: pause data logging"<<endl;
		cout<<"4: start data logging"<<endl;
		cout<<"5: save data file and exit the application"<<endl;
		cout<<"Input your selection:";
		key = getchar();
		if(key == '1')
		{	
			string str;
			cout<<"Input the new filename:";
				//getline(cin,filename);
			cin>>str;
			icm_filename = "icm_";
			icm_filename.append(str);
			icm_filename.append(".txt");
			icm_queue.push(NEW_FILE_EVENT);
		}
		else if(key == '2')
		{
			icm_queue.push(FILE_CLOSE_EVENT);
		}
		else if(key == '3')
		{
			icm_queue.push(LOG_START_EVENT);
		}
		else if(key == '4')
		{
			icm_queue.push(LOG_STOP_EVENT);
		}
		else if(key == '5')
		{
			icm_queue.push(APP_EXIT_EVENT);
			pthread_join(icm_thread, NULL);
			exit(0);
		}
	}
	
	return 0;
}

