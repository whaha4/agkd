# 进阶教程

### 创建自己的控制器(Controller)

首先你需要引入`controller.hpp`，你的控制器需要`public继承`控制器类`Controller`

你的控制器会从`Controller`继承得到
- `out`，一个`float`变量，用来存放控制器的输出
- `set(float x)`, 一个纯虚函数，**需要由你自己重载**，用来进行控制器的计算

你的控制器需要做
- 重载实现`set`函数，并在`set`中对`out`变量进行赋值，存放输出
- 重载实现虚析构器，一般只要设为`default`即可

下面是一个控制器例子，这个例子只是把输入乘以某个系数并输出
```c++
class Mul : public Controller {
public:
    float k = 0;
    
    Mul(float k_) : k(k_) {}
    
    void set(float x) override {
        out = x * k;
    }
    
    ~Mul() override = default;
};
```
随后我们就可以尝试使用它
```c++
Mul mul(2.21);
std::cout << (2 >> mul) << std::endl;
```
可以得到4.42的输出

### 创建自己的执行器(Actuator)
首先你需要引入`actuator.hpp`，你的执行器需要`public继承`执行器类`Actuator`

你的执行器会从`Actuator`继承得到
- `set_directly(float x)`，一个纯虚函数，**需要由你自己重载**，用来调用执行器执行

你的执行器需要做
- 重载实现`set_directly`函数，在该函数中将传入的值输出给物理执行器
- 重载实现虚析构器，一般只要设为`default`即可

下面是一个执行器例子，这个例子只是把输入通过`cout`输出
```c++
class Output : public Actuator {
public:
    void set_directly(float x) override {
        std::cout << "get: " << x << std::endl;
    }
    ~Output() override = default;
};
```
尝试和控制器一同使用
```c++
Mul mul(2.21);
Output output;
2 >> mul >> output;
```
可以得到`get: 4.42`的输出

以上的demo位于[demo/demo4](../demo/demo4.cpp)

### 创建自己的IO接口
请参阅[tutorials/io](io.md)
