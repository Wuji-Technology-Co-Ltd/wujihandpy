# WujihandPy: Python bindings for WujiHandCpp

WujihandPy 是 [WujiHandCpp](https://github.com/Wuji-Technology-Co-Ltd/wujihandcpp) 的 Python 绑定。

提供更简洁、更高效、更易用的接口与灵巧手设备进行交互。

## 安装

已发布至 PyPI，使用 pip 安装即可：

```bash
pip install wujihandpy
```

对于 Linux 用户，需要额外配置 udev 规则以允许非 root 用户访问 USB 设备：

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666"' |
sudo tee /etc/udev/rules.d/95-stm32usb.rules &&
sudo udevadm control --reload-rules &&
sudo udevadm trigger
```

## 支持的 CPU 架构

- x86_64
- ARM64

## 最低系统要求 (Linux)

### glibc 2.28+

使用 glibc 2.28 或更高版本的 Linux 发行版：
- Debian 10+
- Ubuntu 18.10+
- Fedora 29+
- CentOS/RHEL 8+

### Python 3.8+

支持以下 Python 版本：

- Python 3.8-3.14

## 最低系统要求 (Windows)

WujihandPy 目前暂不支持 Windows，我们会尽快推进相关支持。

## 示例代码

所有示例代码均位于 [example](example) 目录下。

## 部分参考 API

### 导入模块

```python
import wujihandpy
import numpy as np
```

### 连接至灵巧手

```python
hand = wujihandpy.Hand()
```

### 读数据

```python
def read_<dataname>(self) -> np.<datatype>
def read_<dataname>(self) -> np.array<datatype> # For bulk-read
```

所有可使用的数据均定义在 `wujihandpy/_core.pyi` 中。

例如，读取灵巧手的上电运行时间(us)：

```python
time = hand.read_system_time()
```

除整手唯一的数据外，每个关节也有自己的数据，数据名称均以 `joint` 作为开头。

例如，读取第1个手指（食指），第0个关节的当前位置数据：

```python
position = hand.finger(1).joint(0).read_joint_position()
```

用一条指令读取多个数据也是可行的，这被称为**批量读 (Bulk-Read)**。

例如，以下指令读取整手所有（20个）关节的当前位置数据：

```python
positions = hand.read_joint_position()
```

进行批量读时，函数返回包含所有数据的 `np.array`：

```python
>>> np.set_printoptions(formatter={'int': lambda x: hex(x)})
>>> print(positions)
[[0x200828 0x2062D3 0x20186B 0x200535]
 [0xDFFE7A 0xBF2318 0x7FC3C2 0x802342]
 [0xE01213 0x900C79 0x3FB49B 0x60191E]
 [0xE04051 0x601EFB 0x4035B6 0x60026B]
 [0xDFD9AC 0x3FA315 0x4015C2 0x5FDF6A]]
```

`read` 函数会阻塞，直到读取完成。保证当函数返回时，读取一定成功。

### 写数据

写数据拥有类似的API，但多了一个参数用于传递目标值：

```python
def write_<dataname>(self, np.<datatype>)
def write_<dataname>(self, np.array<datatype>) # For bulk-write
```

例如，写入单个关节的目标位置数据：

```python
hand.finger(1).joint(0).write_joint_control_position(np.int32(0x800000))
```

关节位置的合法范围为 `[0x000000, 0xFFFFFF]`。

**批量写**数据也是可行的，例如，批量写入第1个手指的目标位置数据：

```python
hand.finger(1).write_joint_control_position(np.int32(0x800000))
```

如果每个关节的目标值不同，可以传入一个包含所有目标值的 `np.array`：

```python
hand.finger(1).write_joint_control_position(
    np.array(
        #   J1        J2        J3        J4
        [0xDFFFFF, 0xBFFFFF, 0x400000, 0x600000],
        dtype=np.int32,
    )
)
```

`write` 函数会阻塞，直到写入完成。保证当函数返回时，写入一定成功。

### 异步读/写

读写函数均有对应的异步版本，函数名以 `_async` 作为后缀。

``` python
async def read_<dataname>_async(self) -> np.<datatype>
async def read_<dataname>_async(self) -> np.array<datatype> # For bulk-read
async def write_<dataname>_async(self, np.<datatype>)
async def write_<dataname>_async(self, np.array<datatype>)  # For bulk-write
```

异步读/写函数同样会异步阻塞，直到读/写完成。保证当函数返回时，读/写一定成功。

### Unchecked 读/写

如果不关心读/写是否成功，可以使用 Unchecked 版本的读/写函数，函数名以 `_unchecked` 作为后缀。

```python
def read_<dataname>_unchecked(self) -> np.<datatype>
def read_<dataname>_unchecked(self) -> np.array<datatype> # For bulk-read
def write_<dataname>_unchecked(self, np.<datatype>)
def write_<dataname>_unchecked(self, np.array<datatype>)  # For bulk-write
```

Unchecked 函数总是立即返回，不会阻塞，通常用于对实时性要求较高的场景。

### Get

如果希望获取以往读/写的结果，可以使用 `get` 系列函数：

```python
def get_<dataname>(self) -> np.<datatype>
def get_<dataname>(self) -> np.array<datatype> # For bulk-read
```

`get` 系列函数同样不会阻塞，它总是立即返回最近一次读取到的数据，无论该数据来自 `read`、`async-read` 还是 `read-unchecked`。 

如果尚未请求过该数据，或请求尚未成功，`get` 函数的返回值是未定义的（通常为0）。

## 性能与优化

WujihandPy 在充分保证易用性的同时，尽可能优化了性能与效率。

我们强烈建议优先使用批量读/写以最大限度地发挥性能。

## 许可证

本项目采用 MIT 许可证，详情见 [LICENSE](LICENSE) 文件。