#include <iostream>
#include <fstream>
#include <assert.h>
#include <vector>
#include <utility>
#include <string.h>
#include <algorithm>
#include <math.h>
#include <time.h>
#include <random>
using namespace std;

#define W 5000000.0 // 信道带宽（Hz）
#define G0 -40 // 路径损耗常数（dB）
#define THETA 4 // 路径损耗指数
#define D0 1 // 参考距离（m）
#define D 100 // 传输距离（m）
#define N0 -174 // 噪声功率谱密度（dB * m / Hz）
#define F 1.0E9 // 服务器CPU频率（Hz）

#define POP_SIZE 30 // 灰狼种群规模
#define EPOCH 1000 // 迭代次数
#define POWER 70.0 // 发射功率（mW）
#define MIN_POS 0.0 // 位置下限
#define MAX_POS 4.0 // 位置上限

default_random_engine rand_eng(time(0)); // 随机数

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

class Wolf {
    public:
        int id;
        vector<Task> taskList;
        vector<double> position; // 位置信息，用于ROV Mapping
        double fitness;

        double calcFitness(); // 计算fitness，见下文
        void ROV(); // ROV Mapping，见下文

        Wolf(int id, vector<Task> taskList) {
            this->id = id;
            this->taskList = taskList;
            // 自动生成随机位置信息
            uniform_real_distribution<double> rand_real(MIN_POS, MAX_POS);
            for(int i=0; i<taskList.size(); i++) {
                this->position.emplace_back(rand_real(rand_eng));
            }
            ROV(); // 生成随机任务序列
            this->fitness = calcFitness(); // 自动计算适应度
        }
        Wolf() { // 默认无参构造函数，用于声明alpha、beta、gamma狼
            this->id = -1;
            this->fitness = INT_MAX;
        }
};
// 计算fitness（makespan）
double Wolf::calcFitness() {
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
// ROV Mapping，更新任务序列
void Wolf::ROV() {
    vector<pair<double, int>> rankedPosition; // 位置信息副本用于排序，不破坏原有的下标顺序
    for(int i=0; i<position.size(); i++)
        rankedPosition.emplace_back(pair<double, int> (position.at(i), i));
    sort(rankedPosition.begin(), rankedPosition.end(),
        []( pair<double, int> a, pair<double, int> b ){ return a.first < b.first; }
    ); // 排序
    vector<Task> newTaskList;
    for(auto i = rankedPosition.begin(); i != rankedPosition.end(); i++) {
        newTaskList.emplace_back(this->taskList.at( (*i).second ));
    } // 取对应下标
    this->taskList = newTaskList;
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
vector<string> iTN {/*"10", "20",*/ "30", "40", "50", "60", "70", "80", "90", "100"}; // 实例任务数量
vector<string> iID {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9"}; // 实例编号
int iRepeatTimes = 5; // 每个实例重复次数

for(auto it_n = iTN.begin(); it_n != iTN.end(); it_n++) { // 实例任务数量循环开始
for(auto it_id = iID.begin(); it_id != iID.end(); it_id++) { // 实例编号循环开始
for(int i_r = 0; i_r < iRepeatTimes; i_r++) { // 实例重复测试开始

    // 读取任务序列
    vector<Task> taskList = readInstanceFile("./TestInstances_3/" + *it_n + "/" + *it_n + "_" + *it_id + ".txt");

    // 记录历代最优值
    vector<double> championFitnessRecord;

    // 开始计时
    struct timeval startTime;
    mingw_gettimeofday(&startTime, NULL);

    // 初始化灰狼种群
    vector<Wolf> population;
    Wolf alphaWolf, betaWolf, deltaWolf, championWolf;
    assert(championWolf.fitness == INT_MAX);
    for(int i=0; i<POP_SIZE; i++)
        population.emplace_back( Wolf(i, taskList) ); // 将个体加入种群
    // 初始设置前三名和历史最佳
    sort( population.begin(), population.end(), [](Wolf a, Wolf b){return a.fitness < b.fitness;} );
    alphaWolf = population.at(0);
    betaWolf = population.at(1);
    deltaWolf = population.at(2);
    if(alphaWolf.fitness < championWolf.fitness)
        championWolf = alphaWolf;
    championFitnessRecord.emplace_back(championWolf.fitness);
    // 初始设置目标位置
    vector<double> targetPosition;
    for(int i=0; i<taskList.size(); i++)
        targetPosition.emplace_back( (alphaWolf.position.at(i) + betaWolf.position.at(i) + deltaWolf.position.at(i)) / 3.0 );

    // 灰狼算法迭代
    for(int epo = 0; epo < EPOCH; epo++) {
        // 对于种群中的每一个个体
        for(auto i = population.begin(); i != population.end(); i++) {
            // 更新参数
            uniform_real_distribution<double> rand_real(0.0, 1.0); // [0, 1]区间内的随机数
            double r_1 = rand_real(rand_eng);
            double r_2 = rand_real(rand_eng);
            double a = 2.0 * (1 - epo/EPOCH);
            double A = a * (2.0 * r_1 - 1.0);
            double C = 2.0 * r_2;

            // 更新位置
            for(int j=0; j<(*i).position.size(); j++) { // 以下标顺序遍历
                double Dist = abs(C * targetPosition.at(j) - (*i).position.at(j));
                double newPosition = targetPosition.at(j) - A * Dist;
                // 界限检查
                if(newPosition < MIN_POS)
                    newPosition = MIN_POS;
                if(newPosition > MAX_POS)
                    newPosition = MAX_POS;
                (*i).position.at(j) = newPosition;
            }

            // 映射任务序列
            (*i).ROV();

            // 更新fitness
            (*i).fitness = (*i).calcFitness();
        }

        // 更新前三名和历史最佳
        sort( population.begin(), population.end(), [](Wolf a, Wolf b){return a.fitness < b.fitness;} );
        alphaWolf = population.at(0);
        betaWolf = population.at(1);
        deltaWolf = population.at(2);
        if(alphaWolf.fitness < championWolf.fitness)
            championWolf = alphaWolf;
        championFitnessRecord.emplace_back(championWolf.fitness);
    }

    // 停止计时
    struct timeval endTime;
    mingw_gettimeofday(&endTime, NULL);
    // 计算运行时间
    double duration = (endTime.tv_sec - startTime.tv_sec)*1000.0 + (endTime.tv_usec - startTime.tv_usec)/1000.0;

    // 记录信息
    resultReport += *it_n + "\t"; // 任务数量
    resultReport += *it_id + '\t'; // 测试实例编号
    resultReport += to_string(i_r) + "\t"; // 重复测试次数
    resultReport += to_string(championWolf.fitness) + "\t"; // 最优makespan
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
    fileOut.open("./Test Result - GWO (Continuous).txt");
    fileOut << resultReport;
    fileOut.close();

    // // 历代最优值记录
    // fileOut.open("./Champion Record - GWO (Continuous).txt");
    // string championRecordReport = "";
    // for(int i=0; i<championFitnessRecord.size(); i++)
    //     championRecordReport += to_string(i) + "\t" + to_string(championFitnessRecord.at(i)) + "\n";
    // fileOut << championRecordReport;
    // fileOut.close();

    // Test
    // for(auto i = population.begin(); i != population.end(); i++) {
    //     for(auto j = (*i).taskList.begin(); j != (*i).taskList.end(); j++)
    //         cout << (*j).id <<' ';
    //     cout << endl;
    //     // for(auto j = (*i).position.begin(); j != (*i).position.end(); j++)
    //     //     cout << (*j) <<' ';
    //     // cout << endl;
    //     cout << (*i).id << ' ' << (*i).fitness << '\n';
    // }
    // cout << "----------\n";
    // for(auto j = alphaWolf.taskList.begin(); j != alphaWolf.taskList.end(); j++)
    //     cout << (*j).id <<' ';
    // cout << endl;
    // cout << alphaWolf.id << ' ' << alphaWolf.fitness << '\n';

    return 0;
}