#include <iostream>
#include <fstream>
#include <assert.h>
#include <vector>
#include <string.h>
#include <algorithm>
#include <math.h>
#include <time.h>
using namespace std;

#define W 5000000.0 // 信道带宽（Hz）
#define G0 -40 // 路径损耗常数（dB）
#define THETA 4 // 路径损耗指数
#define D0 1 // 参考距离（m）
#define D 100 // 传输距离（m）
#define N0 -174 // 噪声功率谱密度（dB * m / Hz）
#define F 1.0E9 // 服务器CPU频率（Hz）

#define POWER 5.0 // 发射功率（mW）

// 由发射功率计算任务传输速率
double R(double power) {
    return W * log(1 + 1.0E-12 * power / 3.981071705534985E-18 / W) / 0.6931471805599453;
}

class Task {
    public:
        int id;
        double dataSize, cyclePerBit;

        Task(int id, double dataSize, double cyclePerBit) {
            this->id = id;
            this->dataSize = dataSize;
            this->cyclePerBit = cyclePerBit;
        }
};

// 计算makespan
double calcFitness(const vector<Task>& taskList) {
    vector<double> t_ready, t_complete;
    double sum_dataSize = 0.0; // 前i个任务的数据量和
    for(auto i = taskList.begin(); i != taskList.end(); i++) {
        sum_dataSize += (*i).dataSize; // 前i个任务的数据量和
        t_ready.emplace_back(sum_dataSize / R(POWER)); // 第i个任务的准备时间
        double t_dispose_i = (*i).cyclePerBit * (*i).dataSize / F; // 第i个任务的执行时间
        // 第i个任务的完成时间
        if(i == taskList.begin()) {
            assert(t_complete.empty()); // i=1
            t_complete.emplace_back( *(t_ready.rbegin()) + t_dispose_i ); // t_ready_i + t_dispose_i
        }
        else {
            assert( ! t_complete.empty()); // i>1
            double t_ready_i = *(t_ready.rbegin()), t_complete_i_1 = *(t_complete.rbegin());
            t_complete.emplace_back(
                (t_ready_i > t_complete_i_1 ? t_ready_i : t_complete_i_1) + t_dispose_i
            ); // max{t_ready_i, t_complete_(i-1)} + t_dispose_i
        }
    }

    assert(t_complete.size() == taskList.size());
    return *(max_element(t_complete.begin(), t_complete.end()));
}

// 读取Instance文件，格式：id - dataSize - cyclePerBit
vector<Task> readInstanceFile(string fileDir) {
    vector<Task> taskList;
    ifstream fileIn;
    fileIn.open(fileDir);
    assert(fileIn); // 已打开

    int lineNum; fileIn >> lineNum;
    for(int i=0; i<lineNum; i++) {
        int id_t; double data_t, cycle_t;
        fileIn >> id_t >> data_t >> cycle_t;
        taskList.emplace_back( Task(id_t, data_t, cycle_t) );
    }

    fileIn.close();
    return taskList;
}

int main() {
    string resultReport = "";

// 实例测试
vector<string> iTN {"10", "20", "30", "40", "50", "60", "70", "80", "90", "100"}; // 实例任务数量
vector<string> iID {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9"}; // 实例编号
int iRepeatTimes = 5; // 每个实例重复次数

for(auto it_n = iTN.begin(); it_n != iTN.end(); it_n++) { // 实例任务数量循环开始
for(auto it_id = iID.begin(); it_id != iID.end(); it_id++) { // 实例编号循环开始
for(int i_r = 0; i_r < iRepeatTimes; i_r++) { // 实例重复测试开始

    // 读取任务序列
    vector<Task> taskList = readInstanceFile("./TestInstances/" + *it_n + "/" + *it_n + "_" + *it_id + ".txt");

    // 开始计时
    struct timeval startTime;
    mingw_gettimeofday(&startTime, NULL);

    // 计算makespan
    double makespan = calcFitness(taskList);

    // 停止计时
    struct timeval endTime;
    mingw_gettimeofday(&endTime, NULL);
    // 计算运行时间
    double duration = (endTime.tv_sec - startTime.tv_sec)*1000.0 + (endTime.tv_usec - startTime.tv_usec)/1000.0;

    // 记录信息
    resultReport += *it_n + "\t"; // 任务数量
    resultReport += *it_id + '\t'; // 测试实例编号
    resultReport += to_string(i_r) + "\t"; // 重复测试次数
    resultReport += to_string(makespan) + "\t"; // 最优makespan
    resultReport += to_string(duration) + "\t"; // 运行时间
    resultReport += "\n";

    // 控制台输出日志
    time_t time_t_now = time(nullptr);
    char* timeStamp = ctime(&time_t_now);
    timeStamp[strlen(timeStamp) - 1] = 0;
    cout << "[" << timeStamp <<"] ";
    cout << "Completed Instance " + *it_n + "_" + *it_id + "_" + to_string(i_r) + ".\n";

} // 实例重复测试结束
} // 实例编号循环结束
} // 实例任务数量循环结束

    // 写入输出文件
    ofstream fileOut;
    fileOut.open("./Test Result - Round Robin.txt");
    fileOut << resultReport;
    fileOut.close();

    return 0;
}