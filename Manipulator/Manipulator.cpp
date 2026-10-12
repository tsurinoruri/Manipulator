#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <stack>
#include <array>
#include <ctime>
#include <random>
#include <cstdlib>
#include <optional>
#include <any>
#include <stdexcept>
#include <variant>
#include <thread>
#include <chrono>
#include <mutex>
#include <iomanip>
#include <tuple>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

template <typename T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi) {
    return (v < lo) ? lo : (hi < v) ? hi : v;
}

//#include <Eigen/Dense>

using namespace std;

class Log {
public:
    void info(string info) {
        cout << info << endl;
    }
};


int randval() {
    return 1 + rand() % 10;
}

void initialize_random() {
    srand(static_cast<unsigned int>(time(NULL)));
}

struct Point
{
    double x;
    double y;
    double z;
};

struct Angle
{
    double angle;
};

//end effector pose
struct Position
{
    Point point;
    Angle end_effector_angle;

    Point getPoint() {
        return this->point;
    }

    Angle getAngle() {
        return this->end_effector_angle;
    }

    void setPoint(double x, double y, double z) {
        this->point = { x, y, z };
    }

    void setAngle(double angle) {
        this->end_effector_angle = { angle };
    }
};

struct JointState {
    double q;      // положение
    double dq;     // скорость
    double ddq;    // ускорение
    double tau;    // момент
    double l;      //длина

    static JointState setJointState() {
        double q;      // положение
        double dq;     // скорость
        double ddq;    // ускорение
        double tau;    // момент
        double l;      //длина

        cin >> q;
        cout << endl;
        cin >> dq;
        cout << endl;
        cin >> ddq;
        cout << endl;
        cin >> tau;
        cout << endl;
        cin >> l;
        cout << endl;

        JointState js;
        js.q = q;
        js.dq = dq;
        js.ddq = ddq;
        js.tau = tau;
        js.l = l;

        return js;
    }
};

struct Joint {
    Angle angle;
    JointState jointState;

    static Joint create_new_joint(JointState& js) {
        Joint j;
        j.angle = { 0 };
        j.jointState = js;
        return j;
    }
};


//класс для работы с матрицами
class Matrix {
public:

    // Функция для создания своей матрицы 
    static vector<vector<int>> createMatrix(int& rows, int& cols) {
        vector<vector<int>> matrix(rows, vector<int>(cols));
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                cout << "Введите " << i << " " << j << " элемент: ";
                cin >> matrix[i][j];
            }
        }
        return matrix;
    }

    //создание еденичной матрицы
    static vector<vector<int>> createSingularMatrix(int& rows, int& cols) {
        vector<vector<int>> matrix(rows, vector<int>(cols));
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (i == j) {
                    matrix[i][j] = 1;
                }
                else {
                    matrix[i][j] = 0;
                }
            }
        }
        return matrix;
    }

    // Функция для создания матрицы с случайными значениями
    static vector<vector<int>> createRandomMatrix(int rows, int cols) {
        vector<vector<int>> matrix(rows, vector<int>(cols));
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                matrix[i][j] = randval();
            }
        }
        return matrix;
    }

    // Сложение матриц
    static vector<vector<int>> matrixSum(const vector<vector<int>>& a, const vector<vector<int>>& b) {
        int rows = a.size();
        int cols = a[0].size();

        // Проверка совместимости размеров
        if (rows != b.size() || cols != b[0].size()) {
            throw invalid_argument("Матрицы нельзя сложить: размеры не совпадают.");
        }

        vector<vector<int>> result(rows, vector<int>(cols));

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i][j] = a[i][j] + b[i][j];
            }
        }

        return result;
    }

    // Умножение матриц
    static vector<vector<int>> matrixMultiply(const vector<vector<int>>& a, const vector<vector<int>>& b) {
        int rowsA = a.size();
        int colsA = a[0].size();
        int rowsB = b.size();
        int colsB = b[0].size();

        // Проверка возможности умножения
        if (colsA != rowsB) {
            throw invalid_argument("Матрицы нельзя перемножить: количество столбцов первой не равно количеству строк второй.");
        }

        vector<vector<int>> result(rowsA, vector<int>(colsB, 0));

        for (int i = 0; i < rowsA; i++) {
            for (int j = 0; j < colsB; j++) {
                for (int k = 0; k < colsA; k++) {
                    result[i][j] += a[i][k] * b[k][j];
                }
            }
        }

        return result;
    }

    //Детерминатор
    static int det(const vector<vector<int>>& a) {
        int n = a.size();

        for (const auto& row : a) {
            if (row.size() != n) {
                throw invalid_argument("Матрица должна быть квадратной для вычисления определителя");
            }
        }

        //Матрица 1×1
        if (n == 1) {
            return a[0][0];
        }

        //Матрица 2×2
        if (n == 2) {
            return a[0][0] * a[1][1] - a[0][1] * a[1][0];
        }

        int determinant = 0;

        for (int j = 0; j < n; j++) {
            vector<vector<int>> minor;
            for (int i = 1; i < n; i++) {
                vector<int> row;
                for (int k = 0; k < n; k++) {
                    if (k != j) {
                        row.push_back(a[i][k]);
                    }
                }
                minor.push_back(row);
            }

            int sign = (j % 2 == 0) ? 1 : -1;
            determinant += sign * a[0][j] * det(minor);
        }

        return determinant;
    }

    //Синус 
    static vector<vector<double>> sinMatrix(const vector<vector<double>>& matrix) {
        vector<vector<double>> result;
        for (const auto& row : matrix) {
            vector<double> newRow;
            for (double val : row) {
                newRow.push_back(sin(val));
            }
            result.push_back(newRow);
        }
        return result;
    }

    //Косинус 
    static vector<vector<double>> cosMatrix(const vector<vector<double>>& matrix) {
        vector<vector<double>> result;
        for (const auto& row : matrix) {
            vector<double> newRow;
            for (double val : row) {
                newRow.push_back(cos(val));
            }
            result.push_back(newRow);
        }
        return result;
    }

};

// -------- 4x4 матрица (row-major) --------
struct Mat4 {
    double m[4][4]{};

    static Mat4 identity() {
        Mat4 I{};
        for (int i = 0; i < 4; ++i) I.m[i][i] = 1.0;
        return I;
    }

    // Умножение матриц
    Mat4 operator*(const Mat4& b) const {
        Mat4 r{};
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) {
                double s = 0.0;
                for (int k = 0; k < 4; ++k)
                    s += m[i][k] * b.m[k][j];
                r.m[i][j] = s;
            }
        return r;
    }
};

// -------- Параметры звена DH --------
struct DHLink {
    double theta;  // угол вокруг z (переменная для вращательного шарнира)
    double d;      // смещение вдоль z
    double a;      // длина звена вдоль x
    double alpha;  // скрутка вокруг x
};

// -------- Матрица перехода для одного звена --------
inline Mat4 dhTransform(const DHLink& L) {
    const double ct = std::cos(L.theta), st = std::sin(L.theta);
    const double ca = std::cos(L.alpha), sa = std::sin(L.alpha);

    Mat4 T{};
    T.m[0][0] = ct;  T.m[0][1] = -st * ca;  T.m[0][2] = st * sa;  T.m[0][3] = L.a * ct;
    T.m[1][0] = st;  T.m[1][1] = ct * ca;  T.m[1][2] = -ct * sa;  T.m[1][3] = L.a * st;
    T.m[2][0] = 0.0;  T.m[2][1] = sa;  T.m[2][2] = ca;  T.m[2][3] = L.d;
    T.m[3][0] = 0.0;  T.m[3][1] = 0.0;  T.m[3][2] = 0.0;  T.m[3][3] = 1.0;
    return T;
}

struct Mat6xN {
    int n;
    double** m = new double*[6];
    
    Mat6xN(int n) : n(n){
        //todo
        for (int i = 0; i < n; i++) {
            m[i] = new double[n];
        }
    }

    ~Mat6xN() {
        for (int i = 0; i < n; ++i) {
            delete m[i];
        }
        delete m;
    }
};

struct Jacobian {
    int n, m;
    int** jacob_matrix = new int*[n];

    Jacobian(int n, int m) : n(n), m(m) {
        for (int i = 0; i < n; i++) {
            jacob_matrix[i] = new int[m];
        }
    }

    void pull() {
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < m; ++j) {
                std::cout << jacob_matrix[i][j] << " ";
            }
            std::cout << std::endl;
        }
    }

    ~Jacobian() {
        for (int i = 0; i < n; ++i) {
            delete jacob_matrix[i];
        }
        delete jacob_matrix;
    }
};


class Manipulator
{
private:
    Position position;
    int const joints_quantity;
    int degree_of_freedom;
    double load_capacity;
    double force_moment;
    vector<Joint> joints;
    stack<Position> pos_memory;
public:
    Manipulator(
        int const joints_quantity,
        int degree_of_freedom,
        double load_capacity,
        double force_moment
    ):joints_quantity(joints_quantity),
        degree_of_freedom(degree_of_freedom),
        load_capacity(load_capacity),
        force_moment(force_moment)
    {
        setHomePos();

        for (int i = 1; i != joints_quantity; i++) {
            JointState js = JointState::setJointState();
            Joint j = Joint::create_new_joint(js);
            joints.push_back(j);
        }
    }


    void setHomePos() {
        this->position.setPoint(0, 0, 0);
        this->position.setAngle(0);
        pos_memory.push(position);
    }

    void moveOn(double dx, double dy, double dz) {
        Point new_point = position.getPoint();
        this->position.setPoint(new_point.x + dx, new_point.y + dy, new_point.z + dz);
        pos_memory.push(position);
    }

    void setPose(double x, double y, double z) {
        this->position.setPoint(x, y, z);
        pos_memory.push(position);
    }

    void back_pos() {
        this->position = pos_memory.top();
        pos_memory.pop();
    }

    Mat4 forwardKinematics(const std::array<DHLink, 6>& base,
        const std::array<double, 6>& q)
    {
        Mat4 T = Mat4::identity();
        for (int i = 0; i < 6; ++i) {
            DHLink L = base[i];
            L.theta = q[i];          // подставляем переменную шарнира
            T = T * dhTransform(L);
        }
        return T;
    }
    
    ~Manipulator() {};
};

namespace manipulator_test_optional {

    std::optional<Manipulator> createManipulator(int joints_quantity, int degree_of_freedom, double load_capacity, double force_moment) {
        if (joints_quantity <= 0 || degree_of_freedom <= 0 || load_capacity <= 0 || force_moment <= 0) {
            return std::nullopt; // Возвращаем пустой optional в случае некорректных параметров
        }
        return Manipulator(joints_quantity, degree_of_freedom, load_capacity, force_moment);
    }

    void test() {
        Manipulator manipulator(6, 6, 100, 10);
        manipulator.setPose(1, 2, 3);
        manipulator.moveOn(1, 1, 1);
		manipulator.back_pos();
        manipulator.forwardKinematics(
            std::array<DHLink, 6>{
            DHLink{ 0, 0, 0, 0 },
                DHLink{ 0, 0, 0, 0 },
                DHLink{ 0, 0, 0, 0 },
                DHLink{ 0, 0, 0, 0 },
                DHLink{ 0, 0, 0, 0 },
                DHLink{ 0, 0, 0, 0 }
        },
            std::array<double, 6>{ 0, 0, 0, 0, 0, 0 }
        );
    }

    void optional_test() {
        auto manipulator_opt = createManipulator(6, 6, 100, 10);
        if (manipulator_opt) {
            Manipulator manipulator = *manipulator_opt;
            manipulator.setPose(1, 2, 3);
            manipulator.moveOn(1, 1, 1);
            manipulator.back_pos();
        } else {
            std::cout << "Некорректные параметры для создания манипулятора." << std::endl;
        }
	}


    std::variant<Manipulator, std::string> createManipulatorVariant(int joints_quantity, int degree_of_freedom, double load_capacity, double force_moment) {
        if (joints_quantity <= 0 || degree_of_freedom <= 0 || load_capacity <= 0 || force_moment <= 0) {
            return "Некорректные параметры для создания манипулятора.";
        }
        return Manipulator(joints_quantity, degree_of_freedom, load_capacity, force_moment);
	}

}

//для функции управления манипулятором с помощью ПИД-регулятора
//namespace для reset
namespace PIDController {

    class PIDController {
    private:
        double kp_, ki_, kd_;
        double dt_;
        double output_min_, output_max_;
        double integral_;
        double prev_error_;
        double prev_derivative_ = 0.0;
        double prev_output_;
    public:
        PIDController(double kp, double ki, double kd, double dt,
            double output_min = -1.0, double output_max = 1.0)
            : kp_(kp), ki_(ki), kd_(kd), dt_(dt),
            output_min_(output_min), output_max_(output_max),
            integral_(0.0), prev_error_(0.0), prev_output_(0.0) {
        }

        // Обновление регулятора
        double update(double setpoint, double measurement) {
            double error = setpoint - measurement;

            // Пропорциональная составляющая
            double proportional = kp_ * error;

            // Интегральная составляющая (с анти-виндапом)
            integral_ += error * dt_;
            integral_ = std::clamp(integral_, output_min_ / ki_, output_max_ / ki_);
            if (std::abs(ki_) > 1e-9) {
                integral_ = std::clamp(integral_, output_min_ / ki_, output_max_ / ki_);
            }
            double integral_term = ki_ * integral_;

            // Дифференциальная составляющая (с фильтром низких частот)
            double derivative = (error - prev_error_) / dt_;
            derivative = 0.8 * derivative + 0.2 * prev_derivative_; // Фильтр
            double derivative_term = kd_ * derivative;

            // Вычисление выходного сигнала
            double output = proportional + integral_term + derivative_term;
            output = std::clamp(output, output_min_, output_max_);

            // Сохранение состояния для следующего шага
            prev_error_ = error;
            prev_derivative_ = derivative;
            prev_output_ = output;

            return output;
        }

        // Сброс интегральной составляющей (например, при смене цели)
        void reset() {
            integral_ = 0.0;
            prev_error_ = 0.0;
            prev_derivative_ = 0.0;
        }

        // Динамическая перенастройка коэффициентов
        void setGains(double kp, double ki, double kd) {
            kp_ = kp;
            ki_ = ki;
            kd_ = kd;
        }
    };
};

namespace PIDController_test {
    void test() {
        PIDController::PIDController pid(1.0, 0.1, 0.05, 0.1, -10.0, 10.0);
        double setpoint = 5.0;
        double measurement = 0.0;
        for (int i = 0; i < 100; ++i) {
            double output = pid.update(setpoint, measurement);
            measurement += output * 0.1; // Простая модель системы
            std::cout << "Step " << i << ": Measurement = " << measurement << ", Output = " << output << std::endl;
        }
	}
}

namespace manipulator_test_variant {
    void test() {
        auto manipulator_variant = manipulator_test_optional::createManipulatorVariant(6, 6, 100, 10);
        if (std::holds_alternative<Manipulator>(manipulator_variant)) {
            Manipulator manipulator = std::get<Manipulator>(manipulator_variant);
            manipulator.setPose(1, 2, 3);
            manipulator.moveOn(1, 1, 1);
            manipulator.back_pos();
        } else {
            std::cout << std::get<std::string>(manipulator_variant) << std::endl;
        }
    }
}

namespace manipulator_simulation {
    namespace MANIPULATOR_2DOF_HPP {
        struct JointState {
            double theta1;
            double theta2;
        };

        struct EndEffectorPose {
            double x;
            double y;
        };

        class Manipulator2DOF {
        public:
            Manipulator2DOF(double l1, double l2) : l1_(l1), l2_(l2) {}

            EndEffectorPose forwardKinematics(const JointState& joints) const {
                double x = l1_ * std::cos(joints.theta1) + l2_ * std::cos(joints.theta1 + joints.theta2);
                double y = l1_ * std::sin(joints.theta1) + l2_ * std::sin(joints.theta1 + joints.theta2);
                return { x, y };
            }

            std::pair<JointState, JointState> inverseKinematics(const EndEffectorPose& target) const {
                double x = target.x;
                double y = target.y;

                double D = (x * x + y * y - l1_ * l1_ - l2_ * l2_) / (2.0 * l1_ * l2_);
                D = std::clamp(D, -1.0, 1.0);

                double theta2_1 = std::acos(D);
                double theta2_2 = -theta2_1;

                double theta1_1 = std::atan2(y, x) - std::atan2(l2_ * std::sin(theta2_1), l1_ + l2_ * std::cos(theta2_1));
                double theta1_2 = std::atan2(y, x) - std::atan2(l2_ * std::sin(theta2_2), l1_ + l2_ * std::cos(theta2_2));

                return {
                    JointState{theta1_1, theta2_1},
                    JointState{theta1_2, theta2_2}
                };
            }

            struct Jacobian {
                double J11, J12, J21, J22;
            };

            Jacobian computeJacobian(const JointState& joints) const {
                double c1 = std::cos(joints.theta1);
                double s1 = std::sin(joints.theta1);
                double c12 = std::cos(joints.theta1 + joints.theta2);
                double s12 = std::sin(joints.theta1 + joints.theta2);

                return {
                    -l1_ * s1 - l2_ * s12, -l2_ * s12,
                     l1_ * c1 + l2_ * c12,  l2_ * c12
                };
            }

        private:
            double l1_, l2_;
        };
    }

    class ManipulatorSimulation {
    public:
        ManipulatorSimulation(double l1, double l2) : manip_(l1, l2) {
            joints_ = { 0.0, 0.0 };
            velocities_ = { 0.0, 0.0 };
        }

        void applyTorques(double tau1, double tau2, double dt) {
            const double damping = 0.1;
            velocities_.theta1 += (tau1 - damping * velocities_.theta1) * dt;
            velocities_.theta2 += (tau2 - damping * velocities_.theta2) * dt;

            joints_.theta1 += velocities_.theta1 * dt;
            joints_.theta2 += velocities_.theta2 * dt;

            joints_.theta1 = std::clamp(joints_.theta1, -M_PI, M_PI);
            joints_.theta2 = std::clamp(joints_.theta2, -M_PI, M_PI);
        }

        MANIPULATOR_2DOF_HPP::JointState getJoints() const { return joints_; }
        MANIPULATOR_2DOF_HPP::EndEffectorPose getEndEffector() const { return manip_.forwardKinematics(joints_); }

    private:
        MANIPULATOR_2DOF_HPP::Manipulator2DOF manip_;
        MANIPULATOR_2DOF_HPP::JointState joints_;
        MANIPULATOR_2DOF_HPP::JointState velocities_;
    };

    void test() {
        const double L1 = 1.0;
        const double L2 = 0.8;
        const double DT = 0.01;

        ManipulatorSimulation robot(L1, L2);

        PIDController::PIDController pid_x(5.0, 0.5, 0.1, DT, -10.0, 10.0);
        PIDController::PIDController pid_y(5.0, 0.5, 0.1, DT, -10.0, 10.0);

        const double center_x = 0.7;
        const double center_y = 0.5;
        const double radius = 0.5;
        double time = 0.0;

        std::cout << std::fixed << std::setprecision(4);
        std::cout << "Time\tTargetX\tTargetY\tActualX\tActualY\tErrorX\tErrorY\n";

        for (int step = 0; step < 1000; ++step) {
            time += DT;

            double target_x = center_x + radius * std::cos(2.0 * M_PI * time / 2.0);
            double target_y = center_y + radius * std::sin(2.0 * M_PI * time / 2.0);
            MANIPULATOR_2DOF_HPP::EndEffectorPose target{ target_x, target_y };

            MANIPULATOR_2DOF_HPP::EndEffectorPose current = robot.getEndEffector();

            double fx = pid_x.update(target_x, current.x);
            double fy = pid_y.update(target_y, current.y);

            MANIPULATOR_2DOF_HPP::JointState joints = robot.getJoints();
            MANIPULATOR_2DOF_HPP::Manipulator2DOF manip(L1, L2);
            auto J = manip.computeJacobian(joints);

            double tau1 = J.J11 * fx + J.J21 * fy;
            double tau2 = J.J12 * fx + J.J22 * fy;

            robot.applyTorques(tau1, tau2, DT);

            if (step % 10 == 0) {
                double error_x = target_x - current.x;
                double error_y = target_y - current.y;
                std::cout << time << "\t"
                    << target_x << "\t" << target_y << "\t"
                    << current.x << "\t" << current.y << "\t"
                    << error_x << "\t" << error_y << "\n";
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(DT * 1000)));
        }
    } 
} 


void work() {
    //manipulator_test_optional::optional_test();
    //manipulator_test_optional::test();
    PIDController_test::test();
    //manipulator_test_variant::test();
    manipulator_simulation::test();
}

int main()
{
    setlocale(LC_ALL, "RUS");

	std::thread t1(work);
	t1.detach();
	std::thread t2(work);
	t2.join();
	std::this_thread::sleep_for(std::chrono::seconds(1));
    work();

	return 0;
}