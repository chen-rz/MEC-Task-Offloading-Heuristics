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

#define POP_SIZE 30 // 粒子群规模
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

// 计算变换序列
vector<pair<int, int>> calcSwapSequence(const vector<Task>& from, const vector<Task>& to) {
    assert(from.size() == to.size());
    vector<pair<int, int>> swapSequence;
    auto emul = from; // 使用副本模拟变换，保证原序列不受影响
    
    for(int i=0; i<to.size(); i++) {
        for(int j=i; j<emul.size(); j++) {
            if(emul.at(j).id == to.at(i).id) { // 需取出Task的id进行比较
                if(i != j) { // 在同一位置则不需要变化
                    swapSequence.emplace_back(pair<int, int> (i, j));
                    swap(emul.at(i), emul.at(j));
                }
                break;
            }
        }
    }

    return swapSequence;
}

class Particle {
    public:
        int id;
        vector<Task> taskList;
        double fitness;
        vector<pair<int, int>> velocity;

        double calcFitness();
        void initVelocity();

        Particle(int id, vector<Task> taskList) {
            this->id = id;
            this->taskList = taskList;
            this->fitness = calcFitness(); // 自动计算适应度
            this->initVelocity(); // 自动生成初始velocity
        }
        Particle() {
            this->id = -1;
            this->fitness = INT_MAX;
        }
};
// 生成初始velocity
void Particle::initVelocity() {
    auto randomTaskList = this->taskList;
    random_shuffle(randomTaskList.begin(), randomTaskList.end());

    this->velocity = calcSwapSequence(this->taskList, randomTaskList);
}
// 计算fitness（makespan）
double Particle::calcFitness() {
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

    // 记录历代最优值
    vector<double> championFitnessRecord;
    
    // 开始计时
    struct timeval startTime;
    mingw_gettimeofday(&startTime, NULL);

    // 初始化粒子群
    vector<Particle> swarm;
    Particle bestParticle, championParticle;
    championParticle.fitness = INT_MAX;
    for(int i=0; i<POP_SIZE; i++) {
        random_shuffle(taskList.begin(), taskList.end()); // 随机个体
        swarm.emplace_back( Particle(i, taskList) ); // 列入种群，自动计算适应度
    }
    // 初始设置第一名和历史最佳
    sort( swarm.begin(), swarm.end(), [](Particle a, Particle b){return a.fitness < b.fitness;} );
    bestParticle = swarm.at(0);
    if(bestParticle.fitness < championParticle.fitness)
        championParticle = bestParticle;

    // 粒子群算法迭代
    for(int epo = 0; epo < EPOCH; epo++) {
        // 对于每一个粒子
        for(auto i = swarm.begin(); i != swarm.end(); i++) {
            // 参数
            double c_1 = 0.5;
            double c_2 = 0.5;
            double c_3 = 0.5;

            // 计算变换序列
            auto bestSwapSequence = calcSwapSequence((*i).taskList, bestParticle.taskList);
            auto championSwapSequence = calcSwapSequence((*i).taskList, championParticle.taskList);

            // 新的velocity
            decltype((*i).velocity) newVelocity;

            uniform_real_distribution<double> rand_real(0.0, 1.0);
            for(auto i_ss = (*i).velocity.begin(); i_ss != (*i).velocity.end(); i_ss++) {
                if(rand_real(rand_eng) < c_1) {
                    newVelocity.emplace_back((*i_ss));
                    swap((*i).taskList.at((*i_ss).first), (*i).taskList.at((*i_ss).second));
                }
            }
            for(auto i_ss = bestSwapSequence.begin(); i_ss != bestSwapSequence.end(); i_ss++) {
                if(rand_real(rand_eng) < c_2) {
                    newVelocity.emplace_back((*i_ss));
                    swap((*i).taskList.at((*i_ss).first), (*i).taskList.at((*i_ss).second));
                }
            }
            for(auto i_ss = championSwapSequence.begin(); i_ss != championSwapSequence.end(); i_ss++) {
                if(rand_real(rand_eng) < c_3) {
                    newVelocity.emplace_back((*i_ss));
                    swap((*i).taskList.at((*i_ss).first), (*i).taskList.at((*i_ss).second));
                }
            }

            (*i).velocity = newVelocity;

            // 更新fitness
            (*i).fitness = (*i).calcFitness();
        }

        // 更新第一名和历史最佳
        sort( swarm.begin(), swarm.end(), [](Particle a, Particle b){return a.fitness < b.fitness;} );
        bestParticle = swarm.at(0);
        if(bestParticle.fitness < championParticle.fitness)
            championParticle = bestParticle;
        championFitnessRecord.emplace_back(championParticle.fitness);
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
    resultReport += to_string(championParticle.fitness) + "\t"; // 最优makespan
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
    fileOut.open("./Test Result - DPSO.txt");
    fileOut << resultReport;
    fileOut.close();

    // 历代最优值记录
    // fileOut.open("./Champion Record - DPSO.txt");
    // string championRecordReport = "";
    // for(int i=0; i<championFitnessRecord.size(); i++)
    //     championRecordReport += to_string(i) + "\t" + to_string(championFitnessRecord.at(i)) + "\n";
    // fileOut << championRecordReport;
    // fileOut.close();

    // Test
    // for(auto i = swarm.begin(); i != swarm.end(); i++) {
    //     assert(! (*i).taskList.empty());
    //     for(auto j = (*i).taskList.begin(); j != (*i).taskList.end(); j++) {
    //         cout << (*j).id <<' ';
    //     }
    //     cout << endl;
    //     cout << (*i).id << ' ' << (*i).fitness << '\n';
    // }

    return 0;
}