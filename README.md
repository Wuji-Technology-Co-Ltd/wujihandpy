# WujihandCpp: A C++ based Wujihand SDK

这是一个使用 C++20 **全新编写的** Wujihand 灵巧手 SDK (Software Development Kit)。

旨在提供更高效、更易用的接口与灵巧手设备进行交互。

警告：这不是一个 "stable" SDK，其接口和功能可能会在未来的版本中发生变化。

## 部分参考 API

### 连接至灵巧手

```cpp
wujihandcpp::device::Hand hand{usb_vid, usb_pid};
```

定义一个 `Hand` 对象，并传入其 USB VID 和 PID 即可连接。

在目前的固件实现中，所有灵巧手的 VID 固定为 `0x0483`，PID 固定为 `0x5740`：

```cpp
wujihandcpp::device::Hand hand{0x0483, 0x5740};
```

### 读数据

```cpp
read<typename Data>() -> typename Data::ValueType / void;
```

所有可使用的数据类型均定义在 `wujihandcpp/data/data.hpp` 中。

例如，读取灵巧手的上电运行时间(us)：

```cpp
uint32_t time = hand.read<data::hand::SystemTime>();
```

除整手唯一的数据外，每个关节也有自己的数据，定义在 `data::joint` 命名空间下。

例如，读取第1个手指（食指），第0个关节的当前位置数据：

```cpp
int32_t position = hand.finger(1).joint(0).read<data::joint::Position>();
```

用一条指令读取整手所有关节的数据也是可行的，这被称为**批量读**：

```cpp
hand.read<data::joint::Position>();
```

需要注意的是，当进行批量读时，`read` 函数的返回值为 `void`。

此时若希望获取读取完成的数据，需要在 `read` 后调用：

```cpp
hand.finger(1).joint(0).get<data::joint::Position>();
```

`read` 函数会阻塞，直到读取完成。保证当函数返回时，读取一定成功。

不同于 `read` 函数，`get` 从不阻塞，它总是立即返回最后一次读取的数据。
若从未请求过数据，`get` 函数的返回值是未定义的。

### 写数据

写数据拥有类似的API，但多了一个参数用于传递目标值：

```cpp
write<typename Data>(Data::ValueType value) -> void;
```

例如，写入单个关节的目标位置数据：

```cpp
hand.finger(1).joint(0).write<data::joint::ControlPosition>(0x8FFFFF);
```

**批量写**数据也是可行的，例如，批量写入第1个手指的目标位置数据：

```cpp
hand.finger(1).write<data::joint::ControlPosition>(0x8FFFFF);
```

`write` 函数会阻塞，直到写入完成。保证当函数返回时，写入一定成功。

## 构建

### 构建 SDK

构建 SDK 需要 CMake 3.16+ 和一个支持 C++20 的编译器。

目前 SDK 仅能在 Linux 上构建，我们会尽快推进 Windows 的支持。

```bash
git clone https://github.com/Wuji-Technology-Co-Ltd/wujihandcpp.git
cd wujihandcpp
mkdir build && cd build
cmake .. && make -j
```

### 构建示例程序

在 `example` 目录下有一些示例程序。 进入某个示例程序目录，例如 `example/simple_test`：
```bash
cd example/simple_test
mkdir build && cd build
cmake .. && make -j
```

运行生成的可执行文件：
```bash
sudo ./simple_test
```