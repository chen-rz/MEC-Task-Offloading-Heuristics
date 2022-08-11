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

#define POP_SIZE 30 // 种群规模
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

        bool operator==(const Task& anotherTask) {
            return this->id == anotherTask.id;
        }

        Task(int id, double dataSize, double cyclePerBit) {
            this->id = id;
            this->dataSize = dataSize;
            this->cyclePerBit = cyclePerBit;
        }
};

class Chromosome {
    public:
        vector<Task> taskList;
        double fitness;

        double calcFitness();

        Chromosome(vector<Task> taskList) {
            this->taskList = taskList;
            this->fitness = calcFitness(); // 自动计算适应度
        }
        Chromosome() {
            this->fitness = INT_MAX;
        }
};
// 计算fitness（makespan）
double Chromosome::calcFitness() {
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

// Davis Crossover
Chromosome crossover(const Chromosome& a, const Chromosome& b) {
    vector<Task> newTaskList;

    uniform_int_distribution<int> rand_start(0, a.taskList.size() - 1); // 随机选择起始下标
    int a_start = rand_start(rand_eng);
    uniform_int_distribution<int> rand_end(a_start, a.taskList.size() - 1); // 随机选择结束下标
    int a_end = rand_end(rand_eng);

    vector<Task> gene;
    gene.assign(a.taskList.begin() + a_start, a.taskList.begin() + a_end + 1); // 取选定的一段

    auto it_b = b.taskList.begin();
    int count_b = 0;

    while(count_b < a_start) { // 选定段之前的
        assert(it_b != b.taskList.end()); // 确认b中仍有待选元素
        if(find(gene.begin(), gene.end(), *it_b) == gene.end()) { // 选定段中没有该元素，即不重复
            newTaskList.emplace_back(*it_b);
            count_b++;
        }
        it_b++;
    }

    newTaskList.insert(newTaskList.end(), gene.begin(), gene.end()); //插入选定段

    while(it_b != b.taskList.end()) { // 选定段之后的
        if(find(gene.begin(), gene.end(), *it_b) == gene.end()) { // 选定段中没有该元素，即不重复
            newTaskList.emplace_back(*it_b);
        }
        it_b++;
    }

    assert(newTaskList.size() == a.taskList.size());
    return Chromosome(newTaskList); // 自动计算了新的适应度
}

// 变异
void mutate(Chromosome& c) {
    uniform_int_distribution<int> rand_mut_num(1, 3);
    int mutationNum = rand_mut_num(rand_eng);
    for(int i=0; i<mutationNum; i++) {
        uniform_int_distribution<int> rand_mut_index(0, c.taskList.size() - 1);
        int mutationIndex_1 = rand_mut_index(rand_eng);
        int mutationIndex_2 = rand_mut_index(rand_eng);
        swap(c.taskList.at(mutationIndex_1), c.taskList.at(mutationIndex_2));
    }
    c.fitness = c.calcFitness(); // 重新计算适应度
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

    // 初始化种群
    vector<Chromosome> population;
    Chromosome championChromosome;
    championChromosome.fitness = INT_MAX;
    for(int i=0; i<POP_SIZE; i++) {
        random_shuffle(taskList.begin(), taskList.end()); // 随机个体
        population.emplace_back( Chromosome(taskList) ); // 列入种群，自动计算适应度
    }
    // 初始设置历史最佳
    sort( population.begin(), population.end(), [](Chromosome a, Chromosome b){return a.fitness < b.fitness;} );
    championChromosome = population.at(0);

    // 遗传算法迭代
    for(int epo = 0; epo < EPOCH; epo++) {
        
        // 选择
        while(population.size() > POP_SIZE) {
            population.pop_back(); // 已经按fitness升序排序，末位淘汰
        }

        // 交叉
        for(int i = 0; i < population.size() - 1; i+=2) {
            population.emplace_back(crossover(population.at(i), population.at(i+1)));
        }

        // 变异
        for(auto i = population.begin(); i != population.end(); i++) {
            uniform_real_distribution<double> rand_real(0.0, 1.0);
            double mutateOrNot = rand_real(rand_eng);
            if(mutateOrNot < 0.15) {
                mutate(*i);
            }
        }

        // 更新历史最佳
        sort( population.begin(), population.end(), [](Chromosome a, Chromosome b){return a.fitness < b.fitness;} );
        championChromosome = population.at(0);
        championFitnessRecord.emplace_back(championChromosome.fitness);
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
    resultReport += to_string(championChromosome.fitness) + "\t"; // 最优makespan
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
    fileOut.open("./Test Result - GA.txt");
    fileOut << resultReport;
    fileOut.close();

    // 历代最优值记录
    // fileOut.open("./Champion Record - GA.txt");
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
    //     cout << (*i).fitness << '\n';
    // }

    return 0;
}