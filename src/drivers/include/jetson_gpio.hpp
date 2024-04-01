#ifndef __JETSON_GPIO__
#define __JETSON_GPIO__

#include <string>
#include <fstream>

class JetsonGPIO
{
public:
    enum Direction
    {
        INPUT,
        OUTPUT
    };

    enum Value
    {
        LOW,
        HIGH
    };
    
    JetsonGPIO(){}
    JetsonGPIO(int pin, JetsonGPIO::Direction direction, JetsonGPIO::Value initial_value);
    ~JetsonGPIO();
    JetsonGPIO& operator=(const JetsonGPIO& t);

    void setValue(JetsonGPIO::Value value);

    int pin;
private:
    const std::string gpio_path = "/sys/class/gpio";
    
    JetsonGPIO::Direction direction;
    JetsonGPIO::Value curr_value;
    std::string pin_path;
};

JetsonGPIO::JetsonGPIO(int pin, JetsonGPIO::Direction direction, JetsonGPIO::Value initial_value)
{
    // set properties
    this->pin = pin;
    this->direction = direction;
    this->curr_value = initial_value;
    this->pin_path = this->gpio_path + "/gpio" + std::to_string(pin);

    // export pin
    std::ofstream export_file(this->gpio_path + "/export");
    export_file << this->pin;
    export_file.close();

    // set direction
    std::ofstream direction_file(this->pin_path + "/direction");
    direction_file << (direction == JetsonGPIO::Direction::INPUT ? "in" : "out");
    direction_file.close();

    // set initial value
    this->setValue(initial_value);
}

JetsonGPIO::~JetsonGPIO()
{
    // unexport pin
    std::ofstream unexport_file(this->gpio_path + "/unexport");
    unexport_file << this->pin;
    unexport_file.close();
}

JetsonGPIO& JetsonGPIO::operator=(const JetsonGPIO& t)
{
    return *this;
}

void JetsonGPIO::setValue(JetsonGPIO::Value value)
{
    std::ofstream value_file(this->pin_path + "/value");
    value_file << value;
    value_file.close();
}

#endif // __JETSON_GPIO__