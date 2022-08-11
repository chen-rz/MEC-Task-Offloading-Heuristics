#include <iostream>
#include <fstream>
#include <assert.h>
#include <vector>
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
#define POWER 5.0 // 发射功率（mW）

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
        double fitness;

        double calcFitness();

        Wolf(int id, vector<Task> taskList) {
            this->id = id;
            this->taskList = taskList;
            this->fitness = calcFitness(); // 自动计算适应度
        }
        Wolf() {
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

// ？
int Dn(int n) {
    int d_n = - (n % 2) + (n + 1) % 2;
    int sum = 0, u = n, d_u = d_n;
    while(u > 0) {
        sum += d_u;
        d_u = - d_u * u;
        u -= 1;
    }
    sum += d_u;
    return sum;
}
double Prob(int u) {
    if(u > 10)
        return 1.0 / u;
    else
        return (u - 1.0) * Dn(u - 2)/Dn(u);
}

// 根据距离更新任务序列
vector<Task> getNewTaskSequence(vector<Task> taskSeq, int distance) {
    auto newTaskSeq = taskSeq;

    // 保证距离范围
    if(distance > taskSeq.size())
        distance = taskSeq.size();
    if(distance < 1)
        distance = 1;

    vector<int> indexes, pickedIndex, marked;
    decltype(taskSeq) que;
    for(int i=0; i<taskSeq.size(); i++)
        indexes.emplace_back(i);
    
    for(int i=0; i<distance; i++) {
        int rIndex = taskSeq.size() - i - 1; // 反向遍历下标
        uniform_int_distribution<int> rand_int(0, rIndex);
        int swapIndex = rand_int(rand_eng); // 随机取一个用来交换
        swap(indexes.at(rIndex), indexes.at(swapIndex));
        pickedIndex.emplace_back(indexes.at(rIndex)); // 记录已选的下标
        que.emplace_back(taskSeq.at(indexes.at(rIndex)));
        marked.emplace_back(0);
    }

    // ？
    for(int i = 0; i < distance - 1; i++) {
        int rIndex = distance - i - 1; // 反向遍历下标
        uniform_int_distribution<int> rand_int(0, rIndex);
        int swapIndex = rand_int(rand_eng);
        if(marked.at(rIndex) == 1)
            continue;
        while(true) {
            if(marked.at(swapIndex) == 0)
                break;
            int firstavailable = distance;
            for(int i=0; i<distance; i++) {
                if(marked.at(i) == 0)
                    firstavailable = i;
            }
            if(firstavailable >= rIndex)
                break;
        }
        swap(que.at(rIndex), que.at(swapIndex));

        uniform_real_distribution<double> rand_p(0.0, 1.0);
        double p = rand_p(rand_eng);
        if(p < Prob(rIndex + 1))
            marked.at(swapIndex) = 1;
    }

    for(int i=0; i<distance; i++)
        newTaskSeq.at(pickedIndex.at(i)) = que.at(i);
    return newTaskSeq;
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

    // 记录历代最优值
    vector<double> championFitnessRecord;

    // 开始计时
    struct timeval startTime;
    mingw_gettimeofday(&startTime, NULL);

    // 初始化灰狼种群
    vector<Wolf> population;
    Wolf alphaWolf, betaWolf, deltaWolf, championWolf;
    championWolf.fitness = INT_MAX;
    for(int i=0; i<POP_SIZE; i++) {
        shuffle(taskList.begin(), taskList.end(), rand_eng); // 随机个体
        population.emplace_back( Wolf(i, taskList) ); // 列入种群，自动计算适应度
    }
    // 初始设置前三名和历史最佳
    sort( population.begin(), population.end(), [](Wolf a, Wolf b){return a.fitness < b.fitness;} );
    alphaWolf = population.at(0);
    betaWolf = population.at(1);
    deltaWolf = population.at(2);
    if(alphaWolf.fitness < championWolf.fitness)
        championWolf = alphaWolf;

    // 灰狼算法迭代
    for(int epo = 0; epo < EPOCH; epo++) {
        // 对于种群中的每一个个体
        for(auto i = population.begin(); i != population.end(); i++) {
            // 更新参数
            double a = 2.0 * (1 - epo/EPOCH);

            // 更新位置
            double Dist = (*i).taskList.size() * a;
            normal_distribution<double> rand_norm(0, 2);
            Dist += rand_norm(rand_eng);
            uniform_int_distribution<int> rand_int(0, 2);
            (*i).taskList = getNewTaskSequence(population.at(rand_int(rand_eng)).taskList, (int)Dist);

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
    fileOut.open("./Test Result - GWO (Discrete, No Distance).txt");
    fileOut << resultReport;
    fileOut.close();

    // // 历代最优值记录
    // fileOut.open("./Champion Record - GWO (Discrete, No Distance).txt");
    // string championRecordReport = "";
    // for(int i=0; i<championFitnessRecord.size(); i++)
    //     championRecordReport += to_string(i) + "\t" + to_string(championFitnessRecord.at(i)) + "\n";
    // fileOut << championRecordReport;
    // fileOut.close();

    // Test
    // for(auto i = population.begin(); i != population.end(); i++) {
    //     assert(! (*i).taskList.empty());
    //     for(auto j = (*i).taskList.begin(); j != (*i).taskList.end(); j++) {
    //         cout << (*j).id <<' ';
    //     }
    //     cout << endl;
    //     cout << (*i).id << ' ' << (*i).fitness << '\n';
    // }

    return 0;
}