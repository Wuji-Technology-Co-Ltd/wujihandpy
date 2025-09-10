# WujihandPy: Python bindings for WujiHandCpp

WujihandPy 是 [WujiHandCpp](https://github.com/Wuji-Technology-Co-Ltd/wujihandcpp) 的 Python 绑定。

提供更简洁、更高效、更易用的接口与灵巧手设备进行交互。

## 安装

WujihandPy 支持 pip 一键安装：

```bash
pip install wujihandpy
```

对于 Linux 用户，需要额外配置 udev 规则以允许非 root 用户访问 USB 设备，可在终端中输入：

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="0483", MODE="0666"' |
sudo tee /etc/udev/rules.d/95-wujihand.rules &&
sudo udevadm control --reload-rules &&
sudo udevadm trigger
```

### 常见错误

若遇到报错 `Could not find a version that satisfies the requirement`，请尝试升级 pip：

```bash
python3 -m pip install --upgrade pip
```

然后使用新的pip重新安装：

```bash
python3 -m pip install wujihandpy
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

关节角度为 `np.float64` 类型，单位为弧度，零点和正方向与 [URDF文件](https://github.com/Wuji-Technology-Co-Ltd/wujihand-urdf) 中定义的相同。

用一条指令读取多个数据也是可行的，这被称为**批量读 (Bulk-Read)**。

例如，以下指令读取整手所有（20个）关节的当前位置数据：

```python
positions = hand.read_joint_position()
```

进行批量读时，函数返回包含所有数据的 `np.ndarray[np.float64]`：

```python
>>> print(positions)
[[ 0.975  0.523  0.271 -0.45 ]
 [ 0.382  0.241 -0.003 -0.275]
 [-0.299  0.329  0.067 -0.286]
 [-0.122  0.228  0.315 -0.178]
 [ 0.205  0.087  0.288 -0.149]]
```

`read` 函数会阻塞，直到读取完成。保证当函数返回时，读取一定成功。

### 写数据

写数据拥有类似的 API，但多了一个参数用于传递目标值：

```python
def write_<dataname>(self, np.<datatype>)
def write_<dataname>(self, np.array<datatype>) # For bulk-write
```

例如，写入单个关节的目标位置数据：

```python
hand.finger(1).joint(0).write_joint_control_position(np.float64(0.8))
```

各关节的合法角度范围可通过以下 API 获取：

```python
upper = (Hand | Finger | Joint).read_joint_upper_limit()
lower = (Hand | Finger | Joint).read_joint_lower_limit()
```

若写入的角度超出合法范围，会被自动限幅至最高/最低值。

**批量写**数据也是可行的，例如，批量为第一个手指写入目标位置数据：

```python
hand.finger(1).write_joint_control_position(np.float64(0.8))
```

如果每个关节的目标值不同，可以传入一个包含所有目标值的 `np.array`：

```python
hand.finger(1).write_joint_control_position(
    np.array(
        #   J1    J2    J3    J4
        [0.8,  0.0,  0.8,  0.8],
        dtype=np.float64,
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

异步接口需 `await`；等待期间不阻塞线程/事件循环，返回时保证读/写已经成功。

### Unchecked 读/写

如果不关心读/写是否成功，可以使用 Unchecked 版本的读/写函数，函数名以 `_unchecked` 作为后缀。

```python
def read_<dataname>_unchecked(self) -> None
def read_<dataname>_unchecked(self) -> None               # For bulk-read
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

### PDO 写

默认的读/写方式均带有缓冲池，积攒一段数据后才进行传输，最高读/写频率无法超过 100Hz。

对于需要高频率实时控制关节位置（如 1kHz）的场景，需使用 PDO 非阻塞写接口。

```python
hand.pdo_write_unchecked(np.float64(0.8))
# 或按 5x4 结构批量发送：
# hand.pdo_write_unchecked(np.array([...], dtype=np.float64))
```

PDO 启用前需特别配置，见 [example/4.pdo.py](example/4.pdo.py)。

## 性能与优化

WujihandPy 在充分保证易用性的同时，尽可能优化了性能与效率。

我们强烈建议优先使用批量读/写以最大限度地发挥性能。

## 许可证

本项目采用 MIT 许可证，详情见 [LICENSE](LICENSE) 文件。
