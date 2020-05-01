#include "ros/ros.h"
#include <cmath>
#include <iostream>
#include <signal.h>
#include <string>
#include <sstream>
#include <airsim_ros_pkgs/profiling_data_srv.h>
#include <airsim_ros_pkgs/start_profiling_srv.h>
#include <rosgraph_msgs/TopicStatistics.h>
#include <std_msgs/Bool.h>
#include <ros/callback_queue.h>
#include "common_mav.h"
#include "Profiling.h"

#include <map>
using namespace std;

#ifdef USE_NVML
#include "nvml.h"
#endif  // USE_NVML
#include <cstdio>
#include <chrono>
#include <ctime>

//global variables
AirsimROSWrapper* g_drone;
string g_mission_status = "failed";
string g_localization_status = "healthy";
float g_coverage = 0;
bool g_end_requested = false;

string g_ns; 
string g_localization_method;
string g_stats_fname;
//profiling variable
std::vector<KeyValuePairStruct> g_highlevel_application_stats;
std::map <std::string, statsStruct> g_topics_stats;
std::map <std::string, statsStruct> g_topics_stats_filterd;
bool g_start_profiling_data = false;

#define MAX_CPUS    1024
#define MAX_PACKAGES    16
#define NUM_RAPL_DOMAINS    4

static int total_cores=0,total_packages=0;
static int package_map[MAX_PACKAGES];

float g_worst_case_power;

/* TODO: on Skylake, also may support  PSys "platform" domain,    */
/* the whole SoC not just the package.                */
/* see dcee75b3b7f025cc6765e6c92ba0a4e59a4d25f4            */
static int detect_cpu(void) {

    FILE *fff;

    int family,model=-1;
    char buffer[BUFSIZ],*result;
    char vendor[BUFSIZ];

    fff=fopen("/proc/cpuinfo","r");
    if (fff==NULL) return -1;

    while(1) {
        result=fgets(buffer,BUFSIZ,fff);
        if (result==NULL) break;

        if (!strncmp(result,"vendor_id",8)) {
            sscanf(result,"%*s%*s%s",vendor);

            if (strncmp(vendor,"GenuineIntel",12)) {
                printf("%s not an Intel chip\n",vendor);
                return -1;
            }
        }

        if (!strncmp(result,"cpu family",10)) {
            sscanf(result,"%*s%*s%*s%d",&family);
            if (family!=6) {
                printf("Wrong CPU family %d\n",family);
                return -1;
            }
        }

        if (!strncmp(result,"model",5)) {
            sscanf(result,"%*s%*s%d",&model);
        }

    }

    fclose(fff);
    return model;
}

static int detect_packages(void) {

    char filename[BUFSIZ];
    FILE *fff;
    int package;
    int i;

    for(i=0;i<MAX_PACKAGES;i++) package_map[i]=-1;

    for(i=0;i<MAX_CPUS;i++) {
        sprintf(filename,"/sys/devices/system/cpu/cpu%d/topology/physical_package_id",i);
        fff=fopen(filename,"r");
        if (fff==NULL) break;
        fscanf(fff,"%d",&package);
        fclose(fff);

        if (package_map[package]==-1) {
            total_packages++;
            package_map[package]=i;
        }

    }

    total_cores=i;

    return 0;
}

struct rapl_sysfs_stats {
	char basename[MAX_PACKAGES][256];
    char event_names[MAX_PACKAGES][NUM_RAPL_DOMAINS][256];
	char filenames[MAX_PACKAGES][NUM_RAPL_DOMAINS][256];
	long long before[MAX_PACKAGES][NUM_RAPL_DOMAINS];
	long long after[MAX_PACKAGES][NUM_RAPL_DOMAINS];
    double energy[MAX_PACKAGES][NUM_RAPL_DOMAINS];
	int valid[MAX_PACKAGES][NUM_RAPL_DOMAINS];
};


static int setup_rapl_sysfs(rapl_sysfs_stats *s) {
	char tempfile[256];
	int i,j;
	FILE *fff;

    int cpu_model;

    memset(s, 0, sizeof(rapl_sysfs_stats));

    cpu_model=detect_cpu();
    if (cpu_model == -1) {
        std::cout << "Cannot detect Intel CPUs.\n";
        return 0;
    }

    detect_packages();

	for(j=0;j<total_packages;j++) {
		i=0;
		sprintf(s->basename[j],"/sys/class/powercap/intel-rapl/intel-rapl:%d",
			j);
		sprintf(tempfile,"%s/name",s->basename[j]);
		fff=fopen(tempfile,"r");
		if (fff==NULL) {
			fprintf(stderr,"\tCould not open %s\n",tempfile);
			return -1;
		}
		fscanf(fff,"%s",s->event_names[j][i]);
		s->valid[j][i]=1;
		fclose(fff);
		sprintf(s->filenames[j][i],"%s/energy_uj",s->basename[j]);

		/* Handle subdomains */
		for(i=1;i<NUM_RAPL_DOMAINS;i++) {
			sprintf(tempfile,"%s/intel-rapl:%d:%d/name",
				s->basename[j],j,i-1);
			fff=fopen(tempfile,"r");
			if (fff==NULL) {
				//fprintf(stderr,"\tCould not open %s\n",tempfile);
				s->valid[j][i]=0;
				continue;
			}
			s->valid[j][i]=1;
			fscanf(fff,"%s",s->event_names[j][i]);
			fclose(fff);
			sprintf(s->filenames[j][i],"%s/intel-rapl:%d:%d/energy_uj",
				s->basename[j],j,i-1);

		}
	}

	/* Gather before values */
	for(j=0;j<total_packages;j++) {
		for(i=0;i<NUM_RAPL_DOMAINS;i++) {
			if (s->valid[j][i]) {
				fff=fopen(s->filenames[j][i],"r");
				if (fff==NULL) {
					fprintf(stderr,"\tError opening %s!\n",s->filenames[j][i]);
				}
				else {
					fscanf(fff,"%lld",&s->before[j][i]);
					fclose(fff);
				}
			}
		}
	}

    return 0;
}


static double run_rapl_sysfs(rapl_sysfs_stats *s) {
    long long value = 0;
    int i,j;
	FILE *fff;

    /* Gather after values */
	for(j=0;j<total_packages;j++) {
		for(i=0;i<NUM_RAPL_DOMAINS;i++) {
			if (s->valid[j][i]) {
				fff=fopen(s->filenames[j][i],"r");
				if (fff==NULL) {
					fprintf(stderr,"\tError opening %s!\n",s->filenames[j][i]);
				}
				else {
					fscanf(fff,"%lld",&s->after[j][i]);
					fclose(fff);
				}
			}
		}
	}

    std::string core("core");
    double e = 0;
	for(j=0;j<total_packages;j++) {
		// printf("\tPackage %d\n",j);
		for(i=0;i<NUM_RAPL_DOMAINS;i++) {
			if (s->valid[j][i]) {
                s->energy[j][i] = ((double)s->after[j][i]-(double)s->before[j][i])/1000000.0;
                if (core == s->event_names[j][i]) {
				    e += s->energy[j][i];;
                }
			}
		}
	}

    return e;
}

struct xpu_sample_stat {
    xpu_sample_stat(): running(false), sum(0) {}

    bool running;
    double sum;
    std::chrono::time_point<std::chrono::system_clock> start;
};


xpu_sample_stat xs_cpu, xs_gpu;
rapl_sysfs_stats rs;


double read_gpu_power_sample(xpu_sample_stat *s = nullptr) {
    if (!s->running) return 0;

    unsigned int gpu_power_mwatts = 0;
    std::ifstream f1("/sys/devices/3160000.i2c/i2c-0/0-0040/iio_device/in_power0_input");
    if (f1.good()) {
        f1 >> gpu_power_mwatts;
        // std::cout << "GPU Power (tx2): " << gpu_power_mwatts << " mWatt\n";
    }
    f1.close();

#ifdef USE_NVML
    //ROS_INFO_STREAM("using NVML");
    unsigned int num_devices = 0;
    if (nvmlDeviceGetCount(&num_devices) != NVML_SUCCESS) {
        std::cout << nvmlDeviceGetCount(&num_devices) << "nvmlDeviceGetCount() failed.\n";
        return 0;
    }

    nvmlDevice_t gpu_0;
    if (nvmlDeviceGetHandleByIndex(0, &gpu_0) != NVML_SUCCESS) {
        std::cout << "nvmlDeviceGetHandleByIndex() failed.\n";
        return 0;
    }

    if (nvmlDeviceGetPowerUsage(gpu_0, &gpu_power_mwatts) != NVML_SUCCESS) {
        std::cout << "nvmlDeviceGetPowerUsage() failed.\n";
        return 0;
    }

    // std::cout << "GPU Power (nvml): " << gpu_power_mwatts << " mWatt\n";
#endif  // USE_NVML

    if (gpu_power_mwatts <= 0) return 0;
    if (s == nullptr) return gpu_power_mwatts;

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - s->start;
    auto e = gpu_power_mwatts / 1000.0 * elapsed_seconds.count();
    s->start = end;
    s->sum += e;
    return e;
}

double read_cpu_power_sample(xpu_sample_stat *s = nullptr) {
    if (!s->running) return 0;

    unsigned int cpu_power_mwatts = 0;
    std::ifstream f1("/sys/devices/3160000.i2c/i2c-0/0-0041/iio_device/in_power1_input");
    if (f1.good()) {
        f1 >> cpu_power_mwatts;
    }
    f1.close();

    if (cpu_power_mwatts <= 0) return 0;
    if (s == nullptr) return cpu_power_mwatts;

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - s->start;
    auto e = cpu_power_mwatts / 1000.0 * elapsed_seconds.count();
    s->start = end;
    s->sum += e;
    return e;
}



void initialize_params() {
}


void output_flight_summary(void){
    stringstream stats_ss;
    float total_energy_consumed = 0;
    
    //Flight Stats dependent metrics
    stats_ss << "{"<<endl;

    // the rest of metrics 
     
    //the follow is only for the compute subsystem, so it doesn not include memory
    //also, I think for dekstop it doesn't include the platform only gather processor data
    for (auto result_el: g_highlevel_application_stats) {
        // only processor/device energy (not the whole platform)
        if(result_el.key == "gpu_compute_energy" || result_el.key == "cpu_compute_energy"){
            // continue; 
            total_energy_consumed += result_el.value; 
        }
        stats_ss<<  "\t\""<< result_el.key<<'"' <<": " << result_el.value<<"," << endl;
    }
     
  
    stats_ss << "\t\"" <<"total_energy_consumed"<<'"'<<":" << total_energy_consumed << "," << endl;
    // topic rates
    stats_ss << "\t\""  <<"topic_statistics"<<'"'<<":{" << endl;
    
    
    //filter out all the empty ones
    for (auto it = std::begin(g_topics_stats); it !=std::end(g_topics_stats); ++it) {

        if (it->second.ctr != 0){
            g_topics_stats_filterd[it->first] = it->second;
        }
    }
    
    for (auto it = std::begin(g_topics_stats_filterd); it !=std::end(g_topics_stats_filterd); ++it) {
        it->second.calc_stats(); 
        stats_ss << "\t\t\"" <<it->first<<'"'<<":{" << endl;
        stats_ss << "\t\t\t\""<<"mean"<<'"'<<":"<< it->second.mean_pub_rate <<","<< endl;
        stats_ss << "\t\t\t\""<<"std"<<'"'<<":"<< it->second.std_pub_rate << ","<< endl;
        stats_ss << "\t\t\t\""<<"std"<<'"'<<":"<< it->second.std_pub_rate << ","<< endl;
        stats_ss << "\t\t\t\""<<"msg_avg_age"<<'"'<<":"<< it->second.stamp_age_mean<< ","<< endl;
        stats_ss << "\t\t\t\""<<"msg_max_age"<<'"'<<":"<< it->second.stamp_age_max<< ","<< endl;
        stats_ss << "\t\t\t\""<<"droppage_rate"<<'"'<<":"<< it->second.mean_droppage_rate << endl <<"\t\t}";

        if (next(it) !=  g_topics_stats_filterd.end()){
                stats_ss << ","<<endl;
        }else{
            stats_ss << endl<<"},"<<endl;
        }
    }
    
    update_stats_file(g_stats_fname, stats_ss.str());
}


bool start_profiling_cb(airsim_ros_pkgs::start_profiling_srv::Request &req, airsim_ros_pkgs::start_profiling_srv::Response &res){
 if (g_start_profiling_data) {
     res.start = true;
 }else{
     res.start = false;
 }
 return true;
}


bool record_profiling_data_cb(airsim_ros_pkgs::profiling_data_srv::Request &req, airsim_ros_pkgs::profiling_data_srv::Response &res)
{ 
    if (g_drone == NULL) {
        ROS_ERROR_STREAM("drone object is not initialized");
        return false; 
    }

    if(req.key == "start_profiling"){  
        xs_gpu.start = xs_cpu.start = std::chrono::system_clock::now();
        xs_gpu.running = xs_cpu.running = true;
        #ifdef USE_INTEL
        setup_rapl_sysfs(&rs);
        #endif // USE_INTEL
    
        //get list of topics 
        ros::master::V_TopicInfo master_topics;
        ros::master::getTopics(master_topics);
        for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
              const ros::master::TopicInfo& info = *it;
              g_topics_stats[info.name] = statsStruct();
              //std::cout << "topic_" << it - master_topics.begin() << ": " << info.name << std::endl;
        }
        g_start_profiling_data = true;
    }else{ 
        g_highlevel_application_stats.push_back(KeyValuePairStruct(req.key, req.value));
    } 
    return true; 
}


void topic_statistics_cb(const rosgraph_msgs::TopicStatistics::ConstPtr& msg) {
    if (g_topics_stats.size() == 0) {//while not populated with the topic, return
        return;
    }
    long long window_start_sec =  msg->window_start.sec;
    long long window_stop_sec =  msg->window_stop.sec;
    long long window_start_nsec =  msg->window_start.nsec;
    long long window_stop_nsec =  msg->window_stop.nsec;
    double window_duration = (window_stop_sec - window_start_sec)+
        (window_stop_nsec - window_start_nsec)/1e9; //the reason to have this in nano second 
                                                    //is for division to be greater than zero
    long long msg_droppage_rate = (msg->dropped_msgs*100)/window_duration;
    long long stamp_age_mean = (msg->stamp_age_mean.toSec())*1e9;
    double stamp_age_max = (msg->stamp_age_max.toSec());

    int pub_rate  = (int) ((double)1.0/(double)msg->period_mean.toSec());
    int pub_rate_sqr = pub_rate*pub_rate; 
    
    if (g_start_profiling_data){
    g_topics_stats[msg->topic].acc(pub_rate, msg_droppage_rate, 
                                   stamp_age_mean, stamp_age_max);
    }
}

void statistics_queue_timer_callback (const ros::TimerEvent& te, ros::CallbackQueue * cbq) {
     cbq->callAvailable(ros::WallDuration());
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "profile_manager");
    ros::NodeHandle nh, nh_statistics_topic;
    ros::ServiceServer record_profiling_data_service = nh.advertiseService("record_profiling_data", record_profiling_data_cb);
    ros::ServiceServer start_profiling = nh.advertiseService("start_profiling", start_profiling_cb);
    
    initialize_params();

    ros::CallbackQueue statistics_queue;
    ros::Subscriber topic_statistics_sub =  
		nh_statistics_topic.subscribe<rosgraph_msgs::TopicStatistics>("/statistics", 20, topic_statistics_cb);
    nh_statistics_topic.setCallbackQueue(&statistics_queue);
    
#ifdef USE_NVML
    if (nvmlInit() != NVML_SUCCESS) {
        std::cout << "nvmlInit() failed.\n";
        return 0;
    }
#endif

    ros::Rate loop_rate(10);

    ros::Timer statistics_timer = nh_statistics_topic.createTimer(ros::Duration(0.5), boost::bind(statistics_queue_timer_callback, _1, &statistics_queue));

    while (ros::ok()) {
        ros::spinOnce();

        read_gpu_power_sample(&xs_gpu);
        #ifndef USE_INTEL
        read_cpu_power_sample(&xs_cpu);
        #endif // NOT USE_INTEL

        loop_rate.sleep();
        ros::spinOnce();
    }

#ifdef USE_NVML
    nvmlShutdown();
#endif  // USE_NVML

    double cpu_compute_energy = 0, gpu_compute_energy = 0;
    gpu_compute_energy = xs_gpu.sum;
    #ifdef USE_INTEL
    cpu_compute_energy = run_rapl_sysfs(&rs);
    #else
    cpu_compute_energy = xs_cpu.sum;
    #endif // USE_INTEL
     
    g_highlevel_application_stats.push_back(
            KeyValuePairStruct("gpu_compute_energy", gpu_compute_energy));

    g_highlevel_application_stats.push_back(
            KeyValuePairStruct("cpu_compute_energy", cpu_compute_energy));
    
    output_flight_summary();
    return 0;
}