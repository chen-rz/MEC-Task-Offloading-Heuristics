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

/*def Dn(n) :
    d_n = - (n % 2) + (n + 1) % 2
    sum = 0
    u = n
    d_u = d_n
    while(u > 0) :
        sum += d_u
        d_u = - d_u * u
        u -= 1
    sum += d_u

    return sum

def Prob1(u) :
    return (u - 1) * Dn(u - 2) / Dn(u) 

def Prob2(u) :
    return 1 / u

def Prob(u) :
    if(u > 10) :
        return Prob2(u)
    else :
        return Prob1(u)*/


// 根据距离更新任务序列
void updateWolf(Wolf* wolf, int distance) {
    vector<Task>* seq = &(wolf->taskList);

    // 保证距离范围
    if(distance > seq->size())
        distance = seq->size();
    if(distance < 1)
        distance = 1;

    vector<int> indexes, pickedindex, que, marked;
    for(int i=0; i<seq->size(); i++) {
        indexes.emplace_back(i);
    }
    
    for(int i=0; i<distance; i++) {
        int curindex = seq->size() - i - 1;
        uniform_int_distribution<int> rnd_int(0, curindex);
        int swapindex = rnd_int(rnd_eng);
        swap(indexes.at(curindex), indexes.at(swapindex));
        pickedindex.emplace_back(indexes.at(curindex));
        que.emplace_back(seq->at(indexes.at(curindex)));
        marked.emplace_back(0);
    }

    for(int i = 0; i<distance-1;i++) {
        int curindex = distance -1-i;
        if(marked.at(curindex) == 1)
            continue;
        while(1){
            uniform_int_distribution<int> rnd_int(0, curindex);
            int swapindex = rnd_int(rnd_eng);
            if(marked.at(swapindex)==0)
                break;
            int firstavailable = distance;
            for(int i=0;i<distance;i++){
                if(marked.at(i)==0)
                    firstavailable = i;
            }
            if(firstavailable>=curindex)
                break;
            
        }
        swap(que.at(curindex),que.at(swapindex));

        uniform_real_distribution<double> rnd_p(0.0,1.0);
        double p = rnd_p(rnd_eng);
        if(p<Prob(curindex+1))
            marked.at(swapindex) = 1;
    }
    for(int i=0;i<distance;i++)
        seq.at(pickedindex.at(i)) = que.at(i);

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
    // 读取任务序列
    vector<Task> taskList = readInstanceFile("./TestInstances/10/10_0.txt");

    // 灰狼种群
    vector<Wolf> population;
    Wolf alphaWolf, betaWolf, deltaWolf, championWolf;
    championWolf.fitness = INT_MAX;
    for(int i=0; i<POP_SIZE; i++) {
        random_shuffle(taskList.begin(), taskList.end()); // 随机个体
        population.emplace_back( Wolf(i, taskList) ); // 列入种群，自动计算适应度
    }
    // 初始设置前三名和历史最佳
    sort( population.begin(), population.end(), [](Wolf a, Wolf b){return a.fitness < b.fitness;} );
    alphaWolf = population.at(0);
    betaWolf = population.at(1);
    deltaWolf = population.at(2);
    if(alphaWolf.fitness < championWolf.fitness)
        championWolf = alphaWolf;
    // 初始设置参数
    double a = 2.0;
    uniform_real_distribution<double> rand_real(0.0, 1.0); // [0, 1]区间内的随机数
    double r_1 = rand_real(rand_eng);
    double r_2 = rand_real(rand_eng);

    // 灰狼算法迭代
    for(int epo = 0; epo < EPOCH; epo++) {
        // 更新位置


    }

    // Test
    for(auto i = population.begin(); i != population.end(); i++) {
        assert(! (*i).taskList.empty());
        for(auto j = (*i).taskList.begin(); j != (*i).taskList.end(); j++) {
            cout << (*j).id <<' ';
        }
        cout << endl;
        cout << (*i).id << ' ' << (*i).fitness << '\n';
    }

    return 0;
}