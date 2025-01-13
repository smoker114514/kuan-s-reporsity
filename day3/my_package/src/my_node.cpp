#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include <fstream>
#include <iostream>
#include <limits>

class Calculator : public rclcpp::Node
{
public:
  Calculator() : Node("calculator")
  {
    pub_result_ = this->create_publisher<std_msgs::msg::Int32>("result", 10);
    pub_history_ = this->create_publisher<std_msgs::msg::String>("history", 10);
  }

  void menu()
  {
    std::cout << "1. 输入数值" << std::endl;
    std::cout << "2. 运算符" << std::endl;
    std::cout << "3. 查看历史记录" << std::endl;
    std::cout << "4. 清空历史记录" << std::endl;
    std::cout << "5. 退出" << std::endl;
  }

  void input_num()
  {
    std::cout << "请输入第一个数：" << std::endl;
    while (!(std::cin >> num1))
    {
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      std::cout << "请输入有效的数字：" << std::endl;
    }
    std::cout << "请输入第二个数：" << std::endl;
    while (!(std::cin >> num2))
    {
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      std::cout << "请输入有效的数字：" << std::endl;
    }
  }

  void perform_operation(double num1, double num2, std::string operation)
  {
    double result;
    if (operation == "+")
    {
      result = num1 + num2;
    }
    else if (operation == "-")
    {
      result = num1 - num2;
    }
    else if (operation == "*")
    {
      result = num1 * num2;
    }
    else if (operation == "/")
    {
      if (num2 == 0)
      {
        std::cout << "除数不能为零" << std::endl;
        return;
      }
      result = num1 / num2;
    }
    else
    {
      std::cout << "无效的运算符" << std::endl;
      return;
    }
    std::ofstream outFile("history.txt", std::ios::app);
    outFile << result << std::endl;
    outFile.close();
    std_msgs::msg::Int32 msg;
    msg.data = result;
    pub_result_->publish(msg);
  }

  void show_history()
  {
    std::ifstream inFile("history.txt");
    std::string line;
    std::string history;
    while (std::getline(inFile, line))
    {
      history += line + "\n";
    }
    inFile.close();
    std::cout << history << std::endl;

    std_msgs::msg::String msg;
    msg.data = history;
    pub_history_->publish(msg);
  }

  double num1, num2;

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_result_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_history_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Calculator>();
  std::string operation;

  while (true)
  {
    node->menu();
    int choice;
    std::cin >> choice;

    if (choice == 1)
    {
      node->input_num();
    }
    else if (choice == 2)
    {
      std::cout << "请输入运算符 (+, -, *, /): ";
      std::cin >> operation;
      node->perform_operation(node->num1, node->num2, operation);
    }
    else if (choice == 3)
    {
      node->show_history();
    }
    else if (choice == 4)
    {
      std::ofstream file1("history.txt", std::ofstream::trunc);
      file1.close();
    }
    else if (choice == 5)
    {
      rclcpp::shutdown();
      return 0;
    }
    else
    {
      std::cout << "无效的选择" << std::endl;
    }
  }

  return 0;
}



